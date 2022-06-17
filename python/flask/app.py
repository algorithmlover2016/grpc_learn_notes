import json
from flask import Flask, request, redirect, url_for, jsonify
from flask_cors import CORS, cross_origin
from markupsafe import escape
from werkzeug.utils import redirect
import base64
import hashlib
import cv2
import numpy
import sys
import re
import logging

app = Flask(__name__)
CORS(app, support_credentials=True)

@app.route("/")
def index():
    return "Index Page!"

@app.route("/again")
def again():
    return "Hello, World Again!"

@app.route("/hello")
def hello():
    return "Hello, World!"

@app.route('/user/<username>')
def show_user_profile(username):
    # show the user profile for that user
    return f'{sys._getframe().f_code.co_name} User %s' % escape(username)

@app.route('/post/<int:post_id>')
def show_post(post_id):
    # show the post with the given id, the id is an integer
    return f'{sys._getframe().f_code.co_name} Post %d' % post_id

@app.route('/path/<path:subpath>')
def show_subpath(subpath):
    # show the subpath after /path/
    return f'{sys._getframe().f_code.co_name} Subpath %s' % escape(subpath)

@app.route('/save_list', methods = ["POST"])
def save_list():
    logging.info(f"recall {sys._getframe().f_code.co_name} begin")
    request_data = request.get_json()
    img = request_data['img']
    shape = request_data["shape"]
    array_img = numpy.reshape(img, shape)
    cv2.imwrite("./data/re_tensor.jpg", array_img)
    logging.info(f"recall {sys._getframe().f_code.co_name} done")
    return jsonify(data = f"recall {sys._getframe().f_code.co_name} successfully")

@app.route('/save_binary_file', methods = ["POST"])
def save_binary_file():
    logging.info(f"recall {sys._getframe().f_code.co_name} begin")
    data = request.get_data()
    # logging.info(data)
    logging.info(hashlib.md5(base64.b64decode(data)).hexdigest())
    with open("./data/re_file.jpg", 'wb') as f:
        f.write(data)
    return jsonify(data = f"recall {sys._getframe().f_code.co_name} successfully")

@app.route('/save_image', methods = ["POST"])
@cross_origin(supports_credentials=True)
def save_ori_image():
    string_regex_rep = re.compile("data:image.*base64,")
    request_data =  request.get_data().decode("utf-8")
    request_data = string_regex_rep.sub("", request_data)
    request_data = base64.b64decode(request_data)
    logging.debug(f"get data type in {sys._getframe().f_code.co_name} is: {type(request_data)}")
    with open("./data/re_image.png", 'wb') as f:
        f.write(request_data)
    return jsonify(data = f"recall {sys._getframe().f_code.co_name} successfully")

@app.route('/save_image/<string:cvid>/<string:ads_id>/<string:frame_id>', methods = ["POST"])
@cross_origin(supports_credentials=True)
def save_image(cvid, frame_id, ads_id):
    videoId = escape(cvid)
    frameId = escape(frame_id)
    adsId = escape(ads_id)
    string_regex_rep = re.compile("data:image.*base64,")
    request_data =  request.get_data().decode("utf-8")
    request_data = string_regex_rep.sub("", request_data)
    request_data = base64.b64decode(request_data)
    logging.debug(f"get data type in {sys._getframe().f_code.co_name} is: {type(request_data)}")
    with open("./data/re_test_image.png", 'wb') as f:
        f.write(request_data)
    ret = True 
    return jsonify(data = f"recall {sys._getframe().f_code.co_name}: {ret}")

@app.route('/save_json', methods = ["POST"])
def save_json():
    request_data =  request.get_json()
    language = request_data['language']
    framework = request_data['framework']

    # two keys are needed because of the nested object
    python_version = request_data['version_info']['python']

    # an index is needed because of the array
    example = request_data['examples'][0]

    boolean_test = request_data['boolean_test']

    logging.info(f"call {sys._getframe().f_code.co_name}", json.dumps(request_data))
    # return jsonify(data = request_data)
    return '''
           The language value is: {}
           The framework value is: {}
           The Python version is: {}
           The item at index 0 in the example list is: {}
           The boolean value is: {}'''.format(language, framework, python_version, example, boolean_test) 


@app.route('/save', methods = ['POST'])
def save(subpath):
    # show the subpath after /path/
    return f'call {sys._getframe().f_code.co_name} Subpath %s' % escape(subpath)

@app.route('/load', methods = ['GET', 'POST'])
def load():
    videoId = ""
    frameId = ""
    adsId = ""
    if request.method == "POST":
        videoId = request.form.get('cvid')
        adsId = request.form.get('adsId', '')
        frameId = request.form.get('frameId', '')
        logging.info(f"{sys._getframe().f_code.co_name} receive looking up POST id: {videoId}:{adsId}:{frameId}")
    else:
        videoId = request.args.get('cvid')
        adsId = request.args.get('adsId', '')
        frameId = request.args.get('frameId', '')
        logging.info(f"{sys._getframe().f_code.co_name} receive looking up GET id: {videoId}:{adsId}:{frameId}")
    return logging.debug(f"call {sys._getframe().f_code.co_name} id: {videoId}:{adsId}:{frameId}")

@app.route('/load/<string:cvid>', defaults = {"ads_id": "", "frame_id": ""})
@app.route('/load/<string:cvid>/<string:ads_id>', defaults = {"frame_id": ""})
@app.route('/load/<string:cvid>/<string:ads_id>/<string:frame_id>')
def load_specified(cvid, frame_id, ads_id):
    videoId = escape(cvid)
    adsId = escape(ads_id)
    frameId = escape(frame_id)
    logging.debug(f"call {sys._getframe().f_code.co_name} id: {videoId}:{adsId}:{frameId}")
    return list_blob(videoId, adsId, frameId)

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s.%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
                            datefmt='%Y-%m-%d:%H:%M:%S',
                            level=logging.DEBUG)
    app.run(host = '0.0.0.0', port = 8889, debug= True)
