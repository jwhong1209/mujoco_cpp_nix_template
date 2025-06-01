#include "save_data.hpp"

using namespace std;

template <typename T>
void SaveData<T>::write_data(bool is_last_data)
{
  if (!fout_)
  {
    std::cerr << "Cannot open file" << std::endl;
    exit(1);
  }

  for (size_t i = 0; i < data_cell_.size(); ++i)
  {
    fout_ << data_cell_[i];
    if (i < data_cell_.size() - 1)
    {
      fout_ << ", ";
    }
  }

  fout_ << std::endl;

  if (is_last_data)
  {
    data_cell_.clear();
  }
}

template <typename T>
void SaveData<T>::save_scalar(const T & val, bool is_last_data)
{
  data_cell_.push_back(val);
  if (is_last_data == true)
  {
    write_data(true);
  }
}

template <typename T>
void SaveData<T>::save_vector(const VecX<T> & vec, bool is_last_data)
{
  data_cell_.insert(data_cell_.end(), vec.data(), vec.data() + vec.size());
  if (is_last_data == true)
  {
    write_data(true);
  }
}

template class SaveData<float>;
template class SaveData<double>;