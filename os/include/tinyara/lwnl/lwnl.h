/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
#pragma once

#include <stdbool.h>
#include <stdint.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define LWNL_PATH "/dev/lwnl"

/* Light-weight netlink domain definition */
#define AF_LWNL 3

/*	Event type */
#define LWNL_ROUTE 1
#define LWNL_NI_NAME_SIZE 7

/*  stack interface name */
#define LWNL_INTF_NAME "None"

#define LWNL_MAX_PARAM 5

#define LWNL_CB_HEADER_LEN (sizeof(lwnl_cb_status) + sizeof(uint32_t))

/* IOCTL commands ***********************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef enum {
	LWNL_FAIL = -1,
	LWNL_SUCCESS,
	LWNL_INVALID_ARGS,
	LWNL_TIMEOUT,
	LWNL_BUSY,
	LWNL_FILE_ERROR,
	LWNL_ALREADY_CONNECTED,
} lwnl_result_e;

typedef struct {
	void *data;
	uint32_t data_len;
	lwnl_result_e res;
	uint8_t name[LWNL_NI_NAME_SIZE];
} lwnl_data;

typedef enum {
	LWNL_DEV_WIFI,
	LWNL_DEV_BLE,
	LWNL_DEV_STACK,
	LWNL_DEV_TYPE_MAX,
} lwnl_dev_type;

typedef struct {
	uint32_t type;
} lwnl_req;

typedef enum {
	LWNL_REQ_COMMON_GETADDRINFO,
} lwnl_req_common;

typedef struct {
	uint8_t name[LWNL_NI_NAME_SIZE];
	lwnl_req req_type;
	uint32_t data_len;
	void *data;
	void *result;
} lwnl_msg;

typedef struct {
	uint8_t count;
	void *param[LWNL_MAX_PARAM];
} lwnl_msg_params;

typedef struct {
	lwnl_dev_type type;
	uint32_t evt;
} lwnl_cb_status;

typedef struct {
	lwnl_cb_status status;
	void *data;
	uint32_t data_len;
	bool md;
} lwnl_cb_data;

struct sockaddr_lwnl {
	lwnl_dev_type dev_type;
};

struct lwnl_lowerhalf_s;
struct lwnl_upperhalf_s;

/* Callback */
typedef CODE void (*lwnl_callback_t)(struct lwnl_lowerhalf_s *dev, lwnl_cb_status status, void *buffer);

struct lwnl_lowerhalf_s {
	struct lwnl_ops_s *ops;
	struct lwnl_upperhalf_s *parent;
	lwnl_callback_t cbk;
	void *priv;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Registrations */
int lwnl_register(struct lwnl_lowerhalf_s *ldev);
int lwnl_unregister(struct lwnl_lowerhalf_s *ldev);
struct netdev;
int lwnl_register_dev(struct netdev *dev);
int lwnl_postmsg(lwnl_dev_type dev, uint32_t evt, void *buffer, int32_t buf_len);
