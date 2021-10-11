
import datetime
import platform
import os
import shutil
import tensorflow as tf
from tensorflow.keras import callbacks, optimizers
TRAIN_USING_GIT = True

mnist = tf.keras.datasets.mnist
(x_train, y_train) , (x_test, y_test) = mnist.load_data()
print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)
x_train, x_test = x_train / 255.0, x_test / 255.0

def create_model():
  return tf.keras.models.Sequential([
    tf.keras.layers.Flatten(input_shape=(28, 28)),
    tf.keras.layers.Dense(512, activation="relu"),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(10, activation="softmax")
  ])

model = create_model()
if TRAIN_USING_GIT:
  model.compile(optimizer = "adam",
                loss = "sparse_categorical_crossentropy",
                metrics = ['accuracy']
  )

  log_dir_fit_root = "logs/fit/"
  shutil.rmtree(log_dir_fit_root)
  log_dir_fit = log_dir_fit_root + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

  tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir = log_dir_fit, histogram_freq = 1)
  model.fit(x = x_train,
            y = y_train,
            epochs = 5,
            validation_data = (x_test, y_test),
            callbacks = [tensorboard_callback]
  )
  os.system(f"tensorboard --logdir {log_dir_fit}")
else:
  batch_size = 64
  train_input_size = x_train.shape[0]
  train_dataset = tf.data.Dataset.from_tensor_slices((x_train, y_train))
  test_dataset = tf.data.Dataset.from_tensor_slices((x_test, y_test))

  train_dataset = train_dataset.shuffle(train_input_size).batch(batch_size)
  test_dataset = test_dataset.batch(batch_size)

  loss_object = tf.keras.losses.SparseCategoricalCrossentropy()
  optimizer = tf.keras.optimizers.Adam()

  train_loss = tf.keras.metrics.Mean("train_loss", dtype = tf.float32)
  train_accuracy = tf.keras.metrics.SparseCategoricalAccuracy("train_accuracy")

  test_loss = tf.keras.metrics.Mean("test_loss", dtype = tf.float32)
  test_accuracy = tf.keras.metrics.SparseCategoricalAccuracy('test_accuracy')

  def train_step(x_train, y_train):
    with tf.GradientTape() as tape:
      predictions = model(x_train, training = True)
      loss = loss_object(y_train, predictions)
    grads = tape.gradient(loss, model.trainable_variables)
    optimizer.apply_gradients(zip(grads, model.trainable_variables))

    train_loss(loss)
    train_accuracy(y_train, predictions)

  def test_step(x_test, y_test):
    predictions = model(x_test)
    loss = loss_object(y_test, predictions)

    test_loss(loss)
    test_accuracy(y_test, predictions)

  current_time = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
  tape_log_dir_root = "./logs/gradient_tape/"
  shutil.rmtree(tape_log_dir_root)

  train_log_dir = tape_log_dir_root + current_time + "/train"
  test_log_dir = tape_log_dir_root + current_time + "/test"
  train_summary_writer = tf.summary.create_file_writer(train_log_dir)
  test_summary_writer = tf.summary.create_file_writer(test_log_dir)

  EPOCHS = 5
  for epoch in range(EPOCHS):
    for (x_train, y_train) in train_dataset:
      train_step(x_train, y_train)
    with train_summary_writer.as_default():
      tf.summary.scalar("loss", train_loss.result(), step = epoch)
      tf.summary.scalar("accuracy", train_accuracy.result(), step = epoch)

    for (x_test, y_test) in test_dataset:
      test_step(x_test, y_test)
    with test_summary_writer.as_default():
      tf.summary.scalar("loss", test_loss.result(), step = epoch)
      tf.summary.scalar("accuracy", test_accuracy.result(), step = epoch)
    
    template = "Epoch {}, Loss: {}, Accuracy: {}, Test Loss: {}, Test Accuracy: {}"
    print(template.format(
      epoch + 1,
      train_loss.result(),
      train_accuracy.result(),
      test_loss.result(),
      test_accuracy.result()
    ))
    train_loss.reset_state()
    train_accuracy.reset_state()
    test_loss.reset_state()
    test_accuracy.reset_state()
  os.system(f"tensorboard --logdir {tape_log_dir_root}")