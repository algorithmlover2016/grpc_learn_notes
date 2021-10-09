# tensorwatch + jupter notebook
import tensorwatch as tw

tw.draw_model(model, [100, 3, 512, 720])

# tensorboardX and tensorboard
from tensorboardX import SummaryWriter
with SummaryWriter("./output/model_logs", comment= "model_name") as w:
    w.add_graph(model, torch.rand(100, 3, 512, 720))
writer = SummaryWriter("./output/model_logs", comment= "model_name")

tensorboard --logdir "log_path"
