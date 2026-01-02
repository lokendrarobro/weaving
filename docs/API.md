# Logs Data Retrieval API Documentation

This API allows you to retrieve KWIS data from four different tables: `roll`, `defect`, `body`, and `job`. You can specify the table name, start date, end date, offset, and length to retrieve the desired data. The API returns JSON responses containing the requested information.

## Endpoint
```
GET http://<IP_Address>:5555/logs
```

## Request Parameters

The API accepts the following parameters in the request body:

| Parameter    | Type    | Description                                                  |
| ------------ | ------- | ------------------------------------------------------------ |
| `table_name` | String  | The name of the table containing the data to be retrieve.       |
| `start_date` | String  | The start date of the date range in the format 'YYYY-MM-DD'. |
| `end_date`   | String  | The end date of the date range in the format 'YYYY-MM-DD'.   |
| `offset`     | Integer | The number of records to be skip before retrieving data.        |
| `length`     | Integer | The maximum number of records to be retrieve.                   |

## API

```bash
curl -X GET -H "Content-Type: application/json" \
-d '{"table_name":"[TABLE_NAME]","start_date":"[YYYY-MM-DD]","end_date":"[YYYY-MM-DD]","offset":["OFF_SET"],"length":[LENGTH]}' \
http://<IP_Address>:5555/logs
```
## Sample Request

```bash
curl -X GET -H "Content-Type: application/json" \
-d '{"table_name":"roll","start_date":"2023-06-25","end_date":"2023-06-29","offset":0,"length":100}' \
http://192.168.1.26:5555/logs
```

## API Response Format

The API responds with a JSON object containing the following fields:

| Field     | Type    | Description                                    |
| --------- | ------- | ---------------------------------------------- |
| `data`    | String  | The data retrieved from the specified table  is in JSON string format.  |
| `message` | String  | A message indicating the status of the request |
| `status`  | Integer | The HTTP status code of the response.          |

### Example Response

```json
{
  "data": "[{\"defect_id\":\"defect_a71e9bc2-4701-49b5-82f7-4c1e1ebce6c2\",\"job_id\":\"job_39a291bd-0f55-49bf-821a-4d03edc13d39\",\"body_id\":\"body_449a0cab-def8-4052-9130-4726cca674dc\",\"robro_roll_id\":\"roll_dd7b2933-09fd-4d5d-acdf-15ceba0e2c97\",\"defect_position_in_roll\":1408.178711,\"defect_top_left_x\":143,\"defect_top_left_y\":1408,\"defect_width\":24,\"defect_height\":13,\"defect_type\":\"0\",\"confidence\":0.032943,\"image_path\":\"images/image_2023_06_22_16_09_42_0.jpg\",\"stopping_command_issued\":0,\"updated_at\":\"2023-06-22 16:09:42\"}]",
  "message": "success",
  "status": 200
}

```

- The `data` field contains the retrieved data as a string in JSON format. It needs to be parsed to access individual records.
- The `message` field indicates the status of the request, which is set to "success" in this example.
- The `status` field represents the HTTP status code of the response, set to 200 (OK) in this case.
 
## The API supports the following table names:

| Table Name | Description           |
| ---------- | --------------------- |
| `roll`       |  contains the details of every rolls.   |
| `defect`     |  contains the details of every defect in roll in a particular job. |
| `body`       |  contains the details of every body cut in roll.   |
| `job`       |  contains the details regarding job set during roll processed.    |

You can specify the desired table name in the request parameters to retrieve data from the corresponding table.

## Error Handling

If an error occurs, the API may respond with a non-200 HTTP status code along with an error message in the `message` field. You should handle such cases accordingly.

To use the API, follow these steps:

1. Make a request to the desired endpoint with the appropriate table name parameter.
2. Check the response status code:
   - If the status code is 200, retrieve the data from the response's `data` field.
    ```json
    {
      "data": "[Roll Data]",
      "message": "success",
      "status": 200
    }
   ```
    - If the status code is non-200, handle the error by extracting the error message from the `message` field.

    ```json 
    {
      "message": "Invalid date format. Expected yyyy-mm-dd.",
      "status": 400
    }
    ```
## Data Retrieval

To retrieve data from the API, follow these steps:

1. Make a request to the API endpoint with the appropriate parameters, including the table name, date range, offset, and length with `GET` method.
2. Upon receiving the response, ensure to parse the `data` field in the response as it contains the retrieved data in JSON format. This will allow you to access the individual records.

