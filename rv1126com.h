#ifndef __RV1126_COM_H__
#define __RV1126_COM_H__

#ifdef __cplusplus
extern "C" {
#endif

int rv1126_com_open(int port, int baud_rate);

int rv1126_com_close(int file_descriptor);

int rv1126_com_open_file(char *device_filename, int baud_rate);

int rv1126_com_set_attr(int file_descriptor, int data_bits, char parity, int stop_bits, int flow_ctrl);

int rv1126_com_set_timeout(int file_descriptor, int timeout);

int rv1126_com_set_baud_rate(int file_descriptor, int baud_rate) ;

int rv1126_com_flush(int file_descriptor);

int rv1126_com_data_available(int file_descriptor, unsigned int timeout_millisec);

int rv1126_com_send(int file_descriptor, char *buffer, size_t data_len);

int rv1126_com_receive(int file_descriptor, char *buffer,size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* __RV1126_COM_H__ */
