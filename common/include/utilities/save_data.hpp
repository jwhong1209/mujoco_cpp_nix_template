#ifndef SAVE_DATA_HPP_
#define SAVE_DATA_HPP_

/* C++ STL */
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "eigen_types.hpp"

template <typename T>
class SaveData
{
private:
  std::ofstream fout_;
  std::vector<T> data_cell_;

public:
  SaveData(std::string & file_name) : fout_(file_name)
  {
    if (!fout_)
    {
      std::cerr << "Cannot open file" << std::endl;
      exit(1);
    }
  };

  ~SaveData()
  {
    if (fout_.is_open())
    {
      fout_.close();
    }
  }

  void write_data(bool is_last_data = false);
  void save_scalar(const T & val, bool is_last_data = false);
  void save_vector(const VecX<T> & vec, bool is_last_data = false);
};

#endif  // SAVE_DATA_HPP_