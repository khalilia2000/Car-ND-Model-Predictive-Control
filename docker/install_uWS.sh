# Install uWebSockets
cd uWebSockets
make 
sudo make install
cd ..
sudo rm -r uWebSockets

# needed to make the ipopt work
sudo ldconfig