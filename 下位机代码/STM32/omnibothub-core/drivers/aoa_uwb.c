#include "include.h"
#include "usart_fc.h"
#include "sbus.h"
#include "gait_math.h"
#include "gps.h"
#include "fifo.h"
#include "mavl.h"
#include "nav.h"
#include "LobotSerialServo.h"
#include "usbd_cdc_vcp.h"
#include "nlink_linktrack_aoa_nodeframe0.h"
#include "nlink_utils.h"



#pragma pack(1)
typedef struct
{
  uint8_t role;
  uint8_t id;
  nint24_t dis;
  int16_t angle;
  uint8_t fp_rssi;
  uint8_t rx_rssi;
  uint8_t reserved[2];
} nltaoa_nodeframe0_node_raw_t;

typedef struct
{
  uint8_t header[2];
  uint16_t frame_length;
  uint8_t role;
  uint8_t id;
  uint32_t local_time;
  uint32_t system_time;
  uint8_t reserved0[4];
  uint16_t voltage;
  uint8_t valid_node_count;
  // nodes...
  // uint8_t checkSum;
} nltaoa_nodeframe0_raw_t;
#pragma pack()

static nltaoa_nodeframe0_raw_t g_frame;

uint8_t Aoa_UnpackData(const uint8_t *data, int  data_length)
{
	g_nltaoa_nodeframe0.cnt_loss=0;
	g_nltaoa_nodeframe0.connect=1;
  if (data_length < g_nltaoa_nodeframe0.fixed_part_size ||
      data[0] != g_nltaoa_nodeframe0.frame_header ||
      data[1] != g_nltaoa_nodeframe0.function_mark)
    return 0;
  int  frame_length = NLINK_PROTOCOL_LENGTH(data);
  if (data_length < frame_length)
    return 0;
  if (!NLINK_VerifyCheckSum(data, frame_length))
    return 0;

  memcpy(&g_frame, data, g_nltaoa_nodeframe0.fixed_part_size);
  g_nltaoa_nodeframe0.result.role = g_frame.role;
  g_nltaoa_nodeframe0.result.id = g_frame.id;
  g_nltaoa_nodeframe0.result.local_time = g_frame.local_time;
  g_nltaoa_nodeframe0.result.system_time = g_frame.system_time;
  g_nltaoa_nodeframe0.result.voltage = g_frame.voltage / MULTIPLY_VOLTAGE;

  g_nltaoa_nodeframe0.result.valid_node_count = g_frame.valid_node_count;
  nltaoa_nodeframe0_node_raw_t raw_node;
  for (int  i = 0; i < g_frame.valid_node_count; ++i)
  {
    TRY_MALLOC_NEW_NODE(g_nltaoa_nodeframe0.result.nodes[i],
                        nltaoa_nodeframe0_node_t)

    memcpy(&raw_node,
           data + g_nltaoa_nodeframe0.fixed_part_size +
               i * sizeof(nltaoa_nodeframe0_node_raw_t),
           sizeof(nltaoa_nodeframe0_node_raw_t));

    nltaoa_nodeframe0_node_t *node = g_nltaoa_nodeframe0.result.nodes[i];
    node->role = (linktrack_role_e)raw_node.role;
    node->id = raw_node.id;
    node->dis = NLINK_ParseInt24(raw_node.dis) / MULTIPLY_DIS;
    node->angle = raw_node.angle / MULTIPLY_ANGLE;
    node->fp_rssi = raw_node.fp_rssi / MULTIPLY_RSSI;
    node->rx_rssi = raw_node.rx_rssi / MULTIPLY_RSSI;
  }
  return 1;
}

nltaoa_nodeframe0_t g_nltaoa_nodeframe0 = {.fixed_part_size = 21,
                                           .frame_header = 0x55,
                                           .function_mark = 0x07};