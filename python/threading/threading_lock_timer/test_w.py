import os
from test_lock import DlisSingleton
import inspect
import logging
# obj = DlisSingleton.get_instance()
# logging.info(f"{__file__}.dlis singleton obj address: {id(obj)}")
def main():
    frame_info = inspect.currentframe()
    function_name = inspect.getframeinfo(frame_info).function
    logging.info(f"begin to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")

    for i in range(3):
        metaExtractorCommand = "python ./test_main.py"
        extractResult = os.system(metaExtractorCommand)
    logging.info(f"end to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")
if __name__ == "__main__":
    main()