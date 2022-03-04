# reference to https://newbedev.com/how-to-log-source-file-name-and-line-number-in-python
import logging
import sys
import os

# reference to https://stackoverflow.com/questions/533048/how-to-log-source-file-name-and-line-number-in-python
# %(pathname)s Full pathname of the source file where the logging call was issued(if available).
# %(filename)s Filename portion of pathname.
# %(module)s Module (name portion of filename).
# %(funcName)s Name of function containing the logging call.
# %(lineno)d Source line number where the logging call was issued (if available).

logging.basicConfig(format='%(asctime)s.%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d:%H:%M:%S',
    level=logging.DEBUG)

logger = logging.getLogger(__name__)
# logger.setLevel(logging.DEBUG)
logger.debug("This is a debug log")
logger.info("This is an info log")
logger.critical("This is critical")
logger.error("An error occurred")

formatter = logging.Formatter('[%(asctime)s.%(msecs)d] %(levelname)-8s [%(pathname)s:%(lineno)s] p[%(process)s] - %(message)s','%Y%m%d %H:%M:%S')
ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
logger.addHandler(ch)

logging.debug("I am sent to standard out. by logging")
logger.debug("I am sent to standard out.")

# refer to 
class PackagePathFilter(logging.Filter):
    def filter(self, record):
        pathname = record.pathname
        record.relativepath = None
        abs_sys_paths = map(os.path.abspath, sys.path)
        for path in sorted(abs_sys_paths, key = len, reverse = True): # longer paths first
            if not path.endswith(os.sep):
                path += os.sep
            if pathname.startswith(path):
                record.relativepath = os.path.relpath(pathname, path)
                break
        return True

formatter = logging.Formatter('[%(asctime)s.%(msecs)d] %(levelname)-8s [%(relativepath)s:%(lineno)s] pp[%(process)s] - %(message)s','%Y%m%d %H:%M:%S')
relativeCH = logging.StreamHandler(sys.stdout)
relativeCH.setLevel(logging.DEBUG)
relativeCH.setFormatter(formatter)
relativeCH.addFilter(PackagePathFilter())
logger.addHandler(relativeCH)
logging.debug("I am sent to standard out. by logging")
logger.debug("I am sent to standard out.")