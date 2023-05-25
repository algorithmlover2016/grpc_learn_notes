import logging
import threading
import os
from threading import Timer
import time
import inspect
# logging_format = "%(asctime)-15s %(levelname)s [%(basename)s:%(lineno)d] %(message)s"
logging_format = "%(asctime)-15s %(levelname)s [%(filename)s:%(lineno)d] %(message)s"
# logging_format = "%(asctime)-15s %(levelname)s [%(pathname)s:%(lineno)d] %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO)
logging.debug("begin to class DLIS")
class DlisSingleton(object):
    __instance = None
    __lock = threading.Lock()
    __update_token_lock = threading.Lock()
    def __init__(self):
        self.access_token = None 

        self.dlis_client_id = os.getenv("DLIS_CLIENT_ID")
        if not self.dlis_client_id:
            self.dlis_client_id = "dafdafdsfdsfsf"
        self.dlis_client_secret = os.getenv("DLIS_CLIENT_SECRET")

        self.tenant_id = os.getenv("TENANT_ID")
        if not self.tenant_id:
            self.tenant_id = "fdsfasffds"
        self.authority = f"https://login.example.com/{self.tenant_id}"
        
        self.dlis_appid =  os.getenv("DLIS_APPID")
        if not self.dlis_appid:
            self.dlis_appid = "dafdsfsfsdfdsfdsf"
        
        self.scope = [f"{self.dlis_appid}/.default"]
        self.refresh_token()

    @staticmethod
    def get_instance():
        if not DlisSingleton.__instance:
            with DlisSingleton.__lock:
                if not DlisSingleton.__instance:
                    DlisSingleton.__instance = DlisSingleton()
        return DlisSingleton.__instance

    def get_access_token(self):
        with self.__update_token_lock:
            return self.access_token
    
      # Function to refresh the token
    def refresh_token(self):
        # Code to acquire a new access token from Azure AD
        result = dict()
        result['access_token'] = "calsls"
        if "access_token" in result:
            new_access_token = result["access_token"]

            # Update the global access_token variable
            with self.__update_token_lock:
                self.access_token = new_access_token
            logging.info("Access token: " + new_access_token)

        else:
            logging.error(f"get DLIS token error:\n{result.get('error')}\n{result.get('error_description')}\n{result.get('correlation_id')}")

        # Schedule the next token refresh
        self.schedule_token_refresh()

    # Function to schedule the token refresh after 10 minutes
    def schedule_token_refresh(self):
        # Schedule the token refresh after 40 minutes (2400 seconds), because the default internal is 60 minutes
        refresh_interval = 2
        refresh_timer = Timer(refresh_interval, self.refresh_token)
        refresh_timer.daemon = True
        refresh_timer.start()
logging.info("begin to interpret main")
def main():
    logging.info(f"begin to run in {__file__}.{inspect.currentframe().f_lineno}: {main.__name__}")
    for i in range(100):
        obj = DlisSingleton.get_instance()
        print(f"get obj {i}")
        time.sleep(.2)
    logging.info(f"end to run in {__file__}.{inspect.currentframe().f_lineno}: {main.__name__}")
logging.debug("end to interpret main")

logging.debug("begin to interpret module")
if __name__ == "__main__":
    logging.debug("begin to main")
    main()
    logging.debug("end to main")
logging.debug("end to interpret module")