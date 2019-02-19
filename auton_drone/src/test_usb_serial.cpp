#include<iostream>
#include<fstream>
#include <vector>
#include <sstream>

#include <stdlib.h>

using namespace std;

vector<float> read_ultrasonics(istream usb_stream) {
  vector<float> sensor_data;
  size_t buffsize = 32;
  char* raw_data_stream = (char*)calloc(buffsize, sizeof(char));
  usb_stream.get(raw_data_stream, buffsize);  // char or char* ?
  stringstream ss (raw_data_stream);
  while (ss.good()) {
    string substr;
    getline(ss, substr, ',');
    float val = stof(substr);
    sensor_data.push_back(val);
  }
}


int main()
{
  char ch;
  ifstream usb_stream;  // connection with arduino
  usb_stream.open("dev/ttyUSB0");  // ACM- or USB-
  ifstream f;
  f.open("/dev/ttyACM0");
  while (f.get(ch)) {
    cout << ch << endl;
  }
  cout << "done" << endl;
  return 0;
}
