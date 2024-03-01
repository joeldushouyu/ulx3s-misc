rm UsbStreamOutTest
iverilog -o UsbStreamOutTest  ./top/usb_stream_out_test.v ./top/usb_stream_out.v
vvp UsbStreamOutTest