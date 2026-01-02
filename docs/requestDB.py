#!/usr/bin/env python
"""
**
 * @file requestDB.py
 * @author Sanyam Bhawsar (sanyam@robrosystems.com)
 * @brief  Script fetches API data, processes JSON, and saves CSV files.
 * @version 1.0.0
 * @date 2023-08-29
 * 
 * 
** @copyright Copyright (c) 2023

"""
#  * @brief This script retrieves data from an API for specified tables within a date range, 
#           converts and cleans the data, and saves it into CSV files using HTTP requests, 
#           JSON processing, and CSV handling.
import requests
import json
import csv

# API endpoint ()
# GET http://<IP_Address>:5555/logs
api_endpoint = "http://192.168.1.48:5555/logs"

# Define date range
start_date = "2023-05-1"
end_date = "2023-08-29"
# List of table names
table_names = ["roll", "defect", "body", "job"]

# The API accepts the following parameters in the request body:

# | Parameter    | Type    | Description                                                  |
# | ------------ | ------- | ------------------------------------------------------------ |
# | `table_name` | String  | The name of the table containing the data to be retrieve.    |
# | `start_date` | String  | The start date of the date range in the format 'YYYY-MM-DD'. |
# | `end_date`   | String  | The end date of the date range in the format 'YYYY-MM-DD'.   |
# | `offset`     | Integer | The number of records to be skip before retrieving data.     |
# | `length`     | Integer | The maximum number of records to be retrieve.                |

# Function to retrieve and save data
def retrieve_and_save_data(table_name):
    # Prepare request payload
    payload = {
        "table_name": table_name,
        "start_date": start_date,
        "end_date": end_date,
        "offset": 0,
        "length": 1000000  # Adjust as needed
    }
    # Make API request
    response = requests.get(api_endpoint, json=payload)
    print(response) 
     # Make API request
    response = requests.get(api_endpoint, json=payload)

    if response.status_code == 200:
        api_response = response.json()
        data_str = api_response["data"]
        cleaned_data_str = data_str.replace('\x00', '')
        try:
            data = json.loads(cleaned_data_str)
            print(json.dumps(data, indent=4))  # Convert back to formatted JSON string
            # CSV file path
            csv_file_path =  f"{table_name}_data.csv"

            # Write data to CSV file
            with open(csv_file_path, mode='w', newline='', encoding='utf-8') as csv_file:
                csv_writer = csv.writer(csv_file)
                # Write header
                header = list(data[0].keys())
                csv_writer.writerow(header)
                # Write data rows
                for item in data:
                    csv_writer.writerow(item.values())
            print(f"CSV file '{csv_file_path}' has been created successfully.")
        except json.JSONDecodeError as e:
            print("Error decoding JSON:", e)

if __name__ == "__main__":
    for table_name in table_names:
        retrieve_and_save_data(table_name)