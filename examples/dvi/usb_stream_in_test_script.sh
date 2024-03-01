rm UsbStreamInTest
iverilog -o UsbStreamInTest  ./top/usb_stream_in_test.v ./top/usb_stream_in.v
vvp UsbStreamInTest