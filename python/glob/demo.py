import glob
import logging
import os
from pathlib import Path

def main():
    # refer to https://stackoverflow.com/questions/3207219/how-do-i-list-all-files-of-a-directory/3215392#3215392
    suffix = ".jpg"
    curPath = Path("./")
    # All files and directories ending with suffix with depth of 1 folders
    logging.info(glob.glob(f"{curPath}/*{suffix}"))
    curPath = Path(__file__)
    logging.info(glob.glob(f"{list(curPath.parents)[-1]}/*/*"))
    logging.info(glob.glob(f"{curPath.parent.parent}/*"))
    logging.info(glob.glob(f"{curPath.parent.parent}/*/*{suffix}"))

    logging.info("*********************Use os.listdir*********************")
    logging.info(os.listdir(curPath.parent))
    logging.info(os.listdir(curPath.parent.parent))
    logging.info(os.listdir(list(curPath.parents)[-2]))

    concern_folder = Path(os.getcwd()).parent.parent
    only_files = [os.path.join(concern_folder, file) for file in os.listdir(concern_folder) if os.path.isfile(os.path.join(concern_folder, file))]
    logging.info(only_files)


    pass

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s.%(msecs)d %(levelname)-8s [%(filename)s:%(lineno)d] %(message)s',
                        datefmt='%Y-%m-%d:%H:%M:%S',
                        level=logging.INFO)
    main()