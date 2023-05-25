from test_lock import DlisSingleton
import logging
logging.info(f"just import singleton of DLIS")
import time
import inspect
def main():
    frame_info = inspect.currentframe()
    function_name = inspect.getframeinfo(frame_info).function
    logging.info(f"begin to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")
    for i in range(10):
        obj = DlisSingleton.get_instance()
        logging.info(f"get obj {i}, with obj address {id(obj)}")
        time.sleep(1)
    logging.info(f"end to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")

if __name__ == "__main__":
    frame_info = inspect.currentframe()
    function_name = inspect.getframeinfo(frame_info).function
    logging.info(f"begin to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")
    main()
    logging.info(f"end to run in {__file__}.{inspect.getframeinfo(frame_info).lineno}: {function_name}")