# Bin Collection System API - Postman Testing Guide

Base URL: `http://127.0.0.1:8000`

---

## 📋 Bin Management Endpoints

### 1. Create Bin

**Endpoint:** `POST http://127.0.0.1:8000/api/bins`

**Request Body:**
```json
{
  "rfid_tag_id": "RFID-001",
  "bin_type": "recycling",
  "location_name": "Kitchen Floor 1",
  "path_to_bin_id": "PATH-K1-001",
  "path_to_collection_id": "PATH-K1-RET",
  "home_pose_x": 2.5,
  "home_pose_y": 3.0,
  "home_pose_z": 0.0,
  "home_pose_qx": 0.0,
  "home_pose_qy": 0.0,
  "home_pose_qz": 0.0,
  "home_pose_qw": 1.0
}
```

**Response (201 Created):**
```json
{
  "id": 1,
  "bin_id": "BIN-A1B2C3D4",
  "rfid_tag_id": "RFID-001",
  "bin_type": "recycling",
  "location_name": "Kitchen Floor 1",
  "path_to_bin_id": "PATH-K1-001",
  "path_to_collection_id": "PATH-K1-RET",
  "home_pose_x": 2.5,
  "home_pose_y": 3.0,
  "home_pose_z": 0.0,
  "home_pose_qx": 0.0,
  "home_pose_qy": 0.0,
  "home_pose_qz": 0.0,
  "home_pose_qw": 1.0,
  "last_collected": null,
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T10:30:00",
  "is_active": true
}
```

---

### 2. Get All Bins

**Endpoint:** `GET http://127.0.0.1:8000/api/bins`

**Query Parameters (Optional):**
- `skip`: Number of records to skip (default: 0)
- `limit`: Maximum records to return (default: 100)
- `active_only`: Return only active bins (default: true)

**Examples:**
```
GET http://127.0.0.1:8000/api/bins
GET http://127.0.0.1:8000/api/bins?skip=0&limit=10
GET http://127.0.0.1:8000/api/bins?active_only=false
```

**Response (200 OK):**
```json
[
  {
    "id": 1,
    "bin_id": "BIN-A1B2C3D4",
    "rfid_tag_id": "RFID-001",
    "bin_type": "recycling",
    "location_name": "Kitchen Floor 1",
    "path_to_bin_id": "PATH-K1-001",
    "path_to_collection_id": "PATH-K1-RET",
    "home_pose_x": 2.5,
    "home_pose_y": 3.0,
    "home_pose_z": 0.0,
    "home_pose_qx": 0.0,
    "home_pose_qy": 0.0,
    "home_pose_qz": 0.0,
    "home_pose_qw": 1.0,
    "last_collected": null,
    "created_at": "2026-02-06T10:30:00",
    "updated_at": "2026-02-06T10:30:00",
    "is_active": true
  },
  {
    "id": 2,
    "bin_id": "BIN-B2C3D4E5",
    "rfid_tag_id": "RFID-002",
    "bin_type": "waste",
    "location_name": "Office Floor 2",
    "path_to_bin_id": "PATH-O2-001",
    "path_to_collection_id": "PATH-O2-RET",
    "home_pose_x": 5.0,
    "home_pose_y": 8.0,
    "home_pose_z": 0.0,
    "home_pose_qx": 0.0,
    "home_pose_qy": 0.0,
    "home_pose_qz": 0.707,
    "home_pose_qw": 0.707,
    "last_collected": null,
    "created_at": "2026-02-06T10:35:00",
    "updated_at": "2026-02-06T10:35:00",
    "is_active": true
  }
]
```

---

### 3. Get Bin by ID

**Endpoint:** `GET http://127.0.0.1:8000/api/bins/{bin_id}`

**Example:**
```
GET http://127.0.0.1:8000/api/bins/BIN-A1B2C3D4
```

**Response (200 OK):**
```json
{
  "id": 1,
  "bin_id": "BIN-A1B2C3D4",
  "rfid_tag_id": "RFID-001",
  "bin_type": "recycling",
  "location_name": "Kitchen Floor 1",
  "path_to_bin_id": "PATH-K1-001",
  "path_to_collection_id": "PATH-K1-RET",
  "home_pose_x": 2.5,
  "home_pose_y": 3.0,
  "home_pose_z": 0.0,
  "home_pose_qx": 0.0,
  "home_pose_qy": 0.0,
  "home_pose_qz": 0.0,
  "home_pose_qw": 1.0,
  "last_collected": null,
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T10:30:00",
  "is_active": true
}
```

**Error Response (404 Not Found):**
```json
{
  "detail": "Bin not found"
}
```

---

### 4. Update Bin

**Endpoint:** `PUT http://127.0.0.1:8000/api/bins/{bin_id}`

**Example:**
```
PUT http://127.0.0.1:8000/api/bins/BIN-A1B2C3D4
```

**Request Body (Partial Update - All fields optional):**
```json
{
  "location_name": "Kitchen Floor 1 - Updated Location",
  "home_pose_x": 3.0,
  "home_pose_y": 3.5
}
```

**Full Update Example:**
```json
{
  "rfid_tag_id": "RFID-001-NEW",
  "bin_type": "mixed_recycling",
  "location_name": "New Location",
  "path_to_bin_id": "PATH-NEW-001",
  "path_to_collection_id": "PATH-NEW-RET",
  "home_pose_x": 10.0,
  "home_pose_y": 15.0,
  "home_pose_z": 0.5,
  "home_pose_qx": 0.1,
  "home_pose_qy": 0.2,
  "home_pose_qz": 0.3,
  "home_pose_qw": 0.9,
  "is_active": true
}
```

**Response (200 OK):**
```json
{
  "id": 1,
  "bin_id": "BIN-A1B2C3D4",
  "rfid_tag_id": "RFID-001",
  "bin_type": "recycling",
  "location_name": "Kitchen Floor 1 - Updated Location",
  "path_to_bin_id": "PATH-K1-001",
  "path_to_collection_id": "PATH-K1-RET",
  "home_pose_x": 3.0,
  "home_pose_y": 3.5,
  "home_pose_z": 0.0,
  "home_pose_qx": 0.0,
  "home_pose_qy": 0.0,
  "home_pose_qz": 0.0,
  "home_pose_qw": 1.0,
  "last_collected": null,
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T11:45:00",
  "is_active": true
}
```

---

### 5. Delete Bin (Soft Delete)

**Endpoint:** `DELETE http://127.0.0.1:8000/api/bins/{bin_id}`

**Example:**
```
DELETE http://127.0.0.1:8000/api/bins/BIN-A1B2C3D4
```

**Response (204 No Content):**
```
(No response body)
```

**Note:** This is a soft delete. The bin's `is_active` field is set to `false` but the record remains in the database.

---

## 📅 Schedule Management Endpoints

### 1. Create Schedule

**Endpoint:** `POST http://127.0.0.1:8000/api/schedules`

#### Daily Schedule Example:

**Request Body:**
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "14:30",
  "frequency": "daily",
  "description": "Daily afternoon collection",
  "enabled": true
}
```

#### Weekly Schedule Example:

**Request Body:**
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "monday 09:00",
  "frequency": "weekly",
  "description": "Weekly Monday morning collection",
  "enabled": true
}
```

#### Custom Cron Schedule Example:

**Request Body:**
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "0 9,15 * * 1-5",
  "frequency": "custom",
  "description": "Weekday mornings and afternoons (9am and 3pm)",
  "enabled": false
}
```

**Response (201 Created):**
```json
{
  "id": 1,
  "schedule_id": "SCH-E5F6G7H8",
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "14:30",
  "frequency": "daily",
  "enabled": true,
  "last_triggered": null,
  "next_trigger": "2026-02-06T14:30:00",
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T10:30:00",
  "description": "Daily afternoon collection"
}
```

**Schedule Time Formats:**

| Frequency | Format | Example |
|-----------|--------|---------|
| `daily` | `HH:MM` | `"14:30"` |
| `weekly` | `DAY HH:MM` | `"monday 09:00"` |
| `custom` | Cron format (5 fields) | `"0 9 * * *"` |

**Valid Days for Weekly:**
- `monday`, `tuesday`, `wednesday`, `thursday`, `friday`, `saturday`, `sunday`

**Cron Format Examples:**
```
"0 9 * * *"        - Every day at 9:00 AM
"30 14 * * 1-5"    - Weekdays at 2:30 PM
"0 9,15 * * *"     - Every day at 9:00 AM and 3:00 PM
"0 8 * * 1"        - Every Monday at 8:00 AM
"0 10 1 * *"       - First day of every month at 10:00 AM
```

---

### 2. Get All Schedules

**Endpoint:** `GET http://127.0.0.1:8000/api/schedules`

**Query Parameters (Optional):**
- `skip`: Number of records to skip (default: 0)
- `limit`: Maximum records to return (default: 100)
- `bin_id`: Filter by specific bin ID

**Examples:**
```
GET http://127.0.0.1:8000/api/schedules
GET http://127.0.0.1:8000/api/schedules?skip=0&limit=10
GET http://127.0.0.1:8000/api/schedules?bin_id=BIN-A1B2C3D4
```

**Response (200 OK):**
```json
[
  {
    "id": 1,
    "schedule_id": "SCH-E5F6G7H8",
    "bin_id": "BIN-A1B2C3D4",
    "schedule_time": "14:30",
    "frequency": "daily",
    "enabled": true,
    "last_triggered": null,
    "next_trigger": "2026-02-06T14:30:00",
    "created_at": "2026-02-06T10:30:00",
    "updated_at": "2026-02-06T10:30:00",
    "description": "Daily afternoon collection"
  },
  {
    "id": 2,
    "schedule_id": "SCH-F7G8H9I0",
    "bin_id": "BIN-A1B2C3D4",
    "schedule_time": "monday 09:00",
    "frequency": "weekly",
    "enabled": true,
    "last_triggered": null,
    "next_trigger": "2026-02-10T09:00:00",
    "created_at": "2026-02-06T10:35:00",
    "updated_at": "2026-02-06T10:35:00",
    "description": "Weekly Monday morning collection"
  }
]
```

---

### 3. Get Schedule by ID

**Endpoint:** `GET http://127.0.0.1:8000/api/schedules/{schedule_id}`

**Example:**
```
GET http://127.0.0.1:8000/api/schedules/SCH-E5F6G7H8
```

**Response (200 OK):**
```json
{
  "id": 1,
  "schedule_id": "SCH-E5F6G7H8",
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "14:30",
  "frequency": "daily",
  "enabled": true,
  "last_triggered": null,
  "next_trigger": "2026-02-06T14:30:00",
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T10:30:00",
  "description": "Daily afternoon collection"
}
```

**Error Response (404 Not Found):**
```json
{
  "detail": "Schedule not found"
}
```

---

### 4. Update Schedule

**Endpoint:** `PUT http://127.0.0.1:8000/api/schedules/{schedule_id}`

**Example:**
```
PUT http://127.0.0.1:8000/api/schedules/SCH-E5F6G7H8
```

#### Update Time Example:

**Request Body:**
```json
{
  "schedule_time": "16:00",
  "description": "Updated to 4pm collection"
}
```

#### Disable Schedule Example:

**Request Body:**
```json
{
  "enabled": false
}
```

#### Change Frequency Example:

**Request Body:**
```json
{
  "schedule_time": "friday 10:00",
  "frequency": "weekly",
  "enabled": true
}
```

#### Full Update Example:

**Request Body:**
```json
{
  "bin_id": "BIN-B2C3D4E5",
  "schedule_time": "18:00",
  "frequency": "daily",
  "description": "Evening collection",
  "enabled": true
}
```

**Response (200 OK):**
```json
{
  "id": 1,
  "schedule_id": "SCH-E5F6G7H8",
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "16:00",
  "frequency": "daily",
  "enabled": true,
  "last_triggered": null,
  "next_trigger": "2026-02-06T16:00:00",
  "created_at": "2026-02-06T10:30:00",
  "updated_at": "2026-02-06T11:50:00",
  "description": "Updated to 4pm collection"
}
```

---

### 5. Delete Schedule

**Endpoint:** `DELETE http://127.0.0.1:8000/api/schedules/{schedule_id}`

**Example:**
```
DELETE http://127.0.0.1:8000/api/schedules/SCH-E5F6G7H8
```

**Response (204 No Content):**
```
(No response body)
```

**Note:** This is a hard delete. The schedule is permanently removed from the database and unregistered from the scheduler.

---

## 🔑 Field Descriptions

### Bin Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `rfid_tag_id` | string | Yes | Unique RFID tag identifier |
| `bin_type` | string | Yes | Type of bin (e.g., "recycling", "waste", "compost") |
| `location_name` | string | Yes | Human-readable location description |
| `path_to_bin_id` | string | Yes | Path ID for robot to reach the bin |
| `path_to_collection_id` | string | Yes | Path ID from bin to collection point |
| `home_pose_x` | float | No | X coordinate of bin's home position (default: 0.0) |
| `home_pose_y` | float | No | Y coordinate of bin's home position (default: 0.0) |
| `home_pose_z` | float | No | Z coordinate of bin's home position (default: 0.0) |
| `home_pose_qx` | float | No | Quaternion X for orientation (default: 0.0) |
| `home_pose_qy` | float | No | Quaternion Y for orientation (default: 0.0) |
| `home_pose_qz` | float | No | Quaternion Z for orientation (default: 0.0) |
| `home_pose_qw` | float | No | Quaternion W for orientation (default: 1.0) |
| `is_active` | boolean | No | Active status (used in UPDATE only) |

### Schedule Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `bin_id` | string | Yes | ID of the bin to collect |
| `schedule_time` | string | Yes | Time specification (format depends on frequency) |
| `frequency` | string | Yes | Schedule frequency: "daily", "weekly", or "custom" |
| `description` | string | No | Human-readable description |
| `enabled` | boolean | No | Whether schedule is active (default: true) |

---

## 📝 Complete Testing Workflow Example

### Step 1: Create Bins

```
POST http://127.0.0.1:8000/api/bins
```
```json
{
  "rfid_tag_id": "RFID-KITCHEN-01",
  "bin_type": "recycling",
  "location_name": "Kitchen Floor 1",
  "path_to_bin_id": "PATH-K1-001",
  "path_to_collection_id": "PATH-K1-RET"
}
```
**Save the returned `bin_id` (e.g., "BIN-A1B2C3D4")**

---

### Step 2: Create Multiple Schedules for Same Bin

#### Schedule 1: Daily Collection
```
POST http://127.0.0.1:8000/api/schedules
```
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "15:00",
  "frequency": "daily",
  "description": "Daily afternoon collection",
  "enabled": true
}
```

#### Schedule 2: Weekly Deep Clean
```
POST http://127.0.0.1:8000/api/schedules
```
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "monday 08:00",
  "frequency": "weekly",
  "description": "Weekly deep clean",
  "enabled": true
}
```

#### Schedule 3: Weekend Collection
```
POST http://127.0.0.1:8000/api/schedules
```
```json
{
  "bin_id": "BIN-A1B2C3D4",
  "schedule_time": "0 10 * * 6,0",
  "frequency": "custom",
  "description": "Weekend collection at 10am",
  "enabled": true
}
```

---

### Step 3: Query All Schedules for a Bin

```
GET http://127.0.0.1:8000/api/schedules?bin_id=BIN-A1B2C3D4
```

**Expected Response:** Returns all 3 schedules

---

### Step 4: Update Bin Location

```
PUT http://127.0.0.1:8000/api/bins/BIN-A1B2C3D4
```
```json
{
  "location_name": "Kitchen Floor 1 - Updated Position",
  "home_pose_x": 3.5,
  "home_pose_y": 4.0
}
```

---

### Step 5: Disable a Schedule Temporarily

```
PUT http://127.0.0.1:8000/api/schedules/SCH-XXXXX
```
```json
{
  "enabled": false
}
```

---

### Step 6: Delete a Schedule

```
DELETE http://127.0.0.1:8000/api/schedules/SCH-XXXXX
```

---

### Step 7: Delete a Bin

```
DELETE http://127.0.0.1:8000/api/bins/BIN-A1B2C3D4
```

---

## 🚨 Common Error Responses

### 400 Bad Request

**Duplicate RFID Tag:**
```json
{
  "detail": "RFID tag already registered"
}
```

**Invalid Time Format:**
```json
{
  "detail": "Invalid time format for daily: invalid_time. Use HH:MM"
}
```

### 404 Not Found

**Bin Not Found:**
```json
{
  "detail": "Bin not found"
}
```

**Schedule Not Found:**
```json
{
  "detail": "Schedule not found"
}
```

### 500 Internal Server Error

**General Server Error:**
```json
{
  "detail": "Internal server error"
}
```

---

## 📌 Notes

1. **bin_id** is automatically generated by the system (format: `BIN-XXXXXXXX`)
2. **schedule_id** is automatically generated by the system (format: `SCH-XXXXXXXX`)
3. **Timestamps** (`created_at`, `updated_at`) are automatically managed
4. **Soft delete** for bins: Record remains but `is_active` is set to `false`
5. **Hard delete** for schedules: Record is permanently removed
6. Multiple schedules can be created for the same bin
7. All endpoints return JSON responses
8. Use `Content-Type: application/json` header for POST/PUT requests
