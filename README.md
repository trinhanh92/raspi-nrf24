# raspi-nrf24
NRF24L01 CPP Library for Raspberry

## Install library
```sh
cd raspi-nrf24/
make
sudo make install
sudo ldconfig -v | grep librf librf24.so.1 -> librf24.so.1.0
```
## Run examples
```sh
cd examples/
make
sudo ./rf_recv     <-- for receiver
or
sudo ./rf_trans    <-- for transmitter
```


