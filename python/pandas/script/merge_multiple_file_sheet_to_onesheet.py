#-*- coding:utf-8 -*-

from typing import OrderedDict
import pandas as pd
import os
from pathlib import Path
import openpyxl
import logging
import shutil
import sys

logging_format = "%(asctime)-15s %(levelname)s [%(pathname)s:%(lineno)d] p[%(process)s]  %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO)

file_postfix = ".xlsx"
target_city = "Suzhou"
merge_sheet_name = "total_data"

curPath = Path(__file__).parent
xlsx_folder = curPath.parent
output_path = xlsx_folder / "output"
if not output_path.exists():
    output_path.mkdir(parents=True, exist_ok=True)

need_columns = OrderedDict({
    "Requisition ID" : str,
    "City" : str,
    "Candidate Name" : str,
    "System ID" : str,
    "Email Address" : str,
    "Compliance Link" : str,
})

def main():
    global output_path
    output_filename = merge_sheet_name + "_" + target_city + file_postfix

    output_path_final = output_path
    output_path /= output_filename + ".tmp"
    output_path_final /= output_filename

    logging.info(output_path)
    logging.info(output_path_final)
    if output_path.exists():
        os.remove(output_path)
        # shutil.rmtree(output_path, ignore_errors=True)
    writer = pd.ExcelWriter(output_path, engine = "openpyxl")

    total_data = None

    xlsx_names = [name for name in os.listdir(xlsx_folder) if name.endswith(file_postfix)]
    total_rows = 0
    filter_total_rows = 0
    for name in xlsx_names:
        filepath = xlsx_folder/name
        xls_obj = pd.ExcelFile(filepath)
        logging.info(filepath)
        sheets = pd.read_excel(xls_obj, sheet_name = None, names = need_columns.keys(), dtype = need_columns)
        file_total_rows = 0
        filter_file_total_rows = 0
        for sheet_name, sheet_value in sheets.items():
            file_total_rows += sheet_value.shape[0]
            total_rows += sheet_value.shape[0]
            logging.info(f"read {filepath} with {sheet_name}")
            logging.info(f"dataFrame Shape: {sheet_value.shape}, the current file rows: {sheet_value.shape[0]}, file_total_row: {file_total_rows}, total rows: {total_rows}")
            # add filter rules, refer to https://www.geeksforgeeks.org/ways-to-filter-pandas-dataframe-by-column-values/
            sheet_value = sheet_value.loc[sheet_value["City"] == target_city]
            # sheet_value = sheet_value.loc[(sheet_value["City"] == target_city) & (other rules)]
            filter_file_total_rows += sheet_value.shape[0]
            filter_total_rows += sheet_value.shape[0]
            logging.info(f"after filter, current sheet shape:{sheet_value.shape}, the current file rows: {sheet_value.shape[0]}, file_total_row: {filter_file_total_rows}, total rows: {filter_total_rows}")
            if total_data is None:
                total_data = sheet_value
            else:
                total_data = pd.concat([total_data, sheet_value], ignore_index = True)

    total_data.to_excel(excel_writer = writer, sheet_name = merge_sheet_name, encoding = "utf-8", index = False)
    writer.save()
    writer.close()
    if output_path_final.exists():
        os.remove(output_path_final)
        # shutil.rmtree(output_path_final, ignore_errors = True)
    output_path.rename(output_path_final)

if __name__ == "__main__":
    # python .\merge_multiple_file_sheet_to_onesheet.py
    # python .\merge_multiple_file_sheet_to_onesheet.py "Beijing"
    # python .\merge_multiple_file_sheet_to_onesheet.py "Shanghai"
    # the xlsx file should be located in ../*.xlsx
    if len(sys.argv) > 1:
        target_city_tmp = sys.argv[1].strip()
        if len(target_city_tmp) > 0:
            target_city = target_city_tmp
    main()