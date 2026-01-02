# Database Operations Script
## Overview

This script is designed to retrieve data from an API endpoint, process JSON data, and save it as CSV files. It allows users to specify table names and a date range for data retrieval. The retrieved data is then cleaned, converted, and stored in CSV format using HTTP requests, JSON processing, and CSV handling.

# Requirements

    Python 3.x
    requests library (pip install requests)

# Usage
Ensure you have Python 3.x installed on your system.
Install the requests library by running the following command:

``` 
pip install requests
```

##  Modify the script as needed:
Set the api_endpoint variable to the appropriate API URL.
Adjust the start_date and end_date variables to define the desired date range.
Modify the table_names list to include the specific table names for which you want to retrieve data.

## Run the script using the following command:

   ``` 
   python requestDB.py
   ```
The script will iterate through the specified table names, fetch data from the API based on the defined date range, clean and process the JSON response, and save the data as CSV files. CSV files will be named as <table_name>_data.csv and will be created in the same directory as the script.

## Important Notes

Ensure that the API endpoint returns JSON data in the expected format for the script to work correctly.
The script assumes that the API endpoint accepts the provided payload structure (including table_name, start_date, end_date, offset, and length).
Error handling for potential network issues or incorrect API responses is not fully implemented in this version of the script.

## Author
```organization: Robro Systems Pvt. Ltd``` 

``` Name: Sanyam Bhawsar```

```Email: sanyam@robrosystems.com```
## Version

```1.0.0 (Released on 2023-08-29)```

## License

This script is protected under copyright law. You are allowed to use and modify it for your own purposes. However, distribution or usage for commercial purposes without proper authorization is prohibited.