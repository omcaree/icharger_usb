
#ifndef __ICHARGER_USB_H
#define __ICHARGER_USB_H

extern "C" {
    #include <libusb-1.0/libusb.h>
}

#include "icharger_data.h"

#include <memory>
#include <vector>

using namespace std;

void error_exit(const char* msg, int rc, ...);

struct icharger_usb_dev;
typedef icharger_usb_dev* icharger_usb_ptr;
typedef vector<icharger_usb_ptr> charger_list;

#define ICHARGER_VENDOR_ID 0x483
#define ICHARGER_PRODUCT_4010_DUO 0x5751

/**
 * @brief The usb_context struct is a simple RAII wrapper around the libusb_context - its expected
 * that there is only a single one of these in the app.  
 * @see AppController.
 */
struct usb_context { 
	usb_context();
	~usb_context();

	libusb_context* ctx;
	int result;

	operator bool () {
		return result == 0;
	}
};

/**
 * @brief The icharger_usb_dev struct is the actual device controller - it implements the required
 * MODBUS protocol over USB-HID by using libusb.  It is instantiated after the DeviceRegistry
 * uses the all_chargers method to find a recognized charger.
 */
struct icharger_usb_dev {
    icharger_usb_dev(libusb_device* d);
    ~icharger_usb_dev();
   
    int acquire();
 
    int vendorId() const;
    int productId() const;
    
    string serialNumber();
    string manufacturer();
    string product();
    
    int clear_halt(unsigned char endpoint);
    int reset();
    
    ModbusRequestError get_device_only(device_only* output);	
    ModbusRequestError get_channel_status(int channel /* 0 or 1 */, channel_status* output);
    ModbusRequestError get_system_storage(system_storage* output);
	ModbusRequestError get_memory(memory* output);
	ModbusRequestError set_memory(memory* mem);
	ModbusRequestError set_selected_memory(int mem_index);
	ModbusRequestError get_charge_current(u16 *current);
	ModbusRequestError set_charge_current(float currentAmps);
	ModbusRequestError set_discharge_current(float currentAmps);
    ModbusRequestError order(OrderAction action, Channel ch, ProgramType pt, int selected_mem_index);
    
    static charger_list all_chargers(libusb_context* ctx, int vendor, int product, string serial = "");
    
private:
    libusb_device* device;
    libusb_device_handle* handle;
    libusb_device_descriptor descriptor;
    int timeout_ms;
    int configuration;
    
    string descriptor_str(uint8_t desc_idx);
    
    int usb_data_transfer(unsigned char endpoint_address,
                          char* data,
                          int length,
                          int* total_transferred = 0);
    
    ModbusRequestError modbus_request(char func_code, char* input, char *output);
    ModbusRequestError read_request(char func_code, int base_addr, int num_registers, char* dest);
    ModbusRequestError write_request(int base_addr, int num_registers, char* dest);
};

#endif

