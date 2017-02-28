#ifndef UTILITIES
#define UTILITIES

#include "wrap_eigen.hpp"

#define SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }

namespace sejong{
    static std::list< std::string > gs_fileName_string; //global & static

    // Box Muller Transform
    double generator_white_noise(double min, double var);

    std::string pretty_string(double vv);
    std::string pretty_string(sejong::Vector const & vv);
    std::string pretty_string(sejong::Quaternion const & qq);
    std::string pretty_string(sejong::Matrix const & mm, std::string const & prefix);

    
    void pretty_print(sejong::Vector const & vv, std::ostream & os,
                      std::string const & title, std::string const & prefix="", bool nonl = false);
    void pretty_print(const std::vector<double> & _vec, const char* title);
    void pretty_print(const double * _vec, const char* title, int size);

    inline void pretty_print(Eigen::Vector3d const & vv, std::ostream & os,
                             std::string const & title, std::string const & prefix, bool nonl = false)
    {
        pretty_print(static_cast<sejong::Vector const &>(vv), os, title, prefix, nonl);
    }
  
    void pretty_print(sejong::Quaternion const & qq, std::ostream & os,
                      std::string const & title, std::string const & prefix="", bool nonl = false);
  
    void pretty_print(sejong::Matrix const & mm, std::ostream & os,
                      std::string const & title, std::string const & prefix ="",
                      bool vecmode = false, bool nonl = false);
    void saveVectorSequence(const std::vector<sejong::Vector> & seq, std::string _name);
    void printVectorSequence(const std::vector<sejong::Vector> & seq, std::string _name);

    
    void saveVector(const sejong::Vector & _vec,      std::string _name);
    void saveVector(const std::vector<double> & _vec, std::string _name);
    void saveVector(double * _vec,                    std::string _name, int size);
    void saveValue(double _value,                     std::string _name);

    void cleaning_file(std::string _file_name, std::string & ret_file);

    void sqrtm(const sejong::Matrix & mt, sejong::Matrix & sqrt_mt);

    double smooth_changing(double ini, double end, double moving_duration, double curr_time);
    double smooth_changing_vel(double ini, double end, double moving_duration, double curr_time);
    double smooth_changing_acc(double ini, double end, double moving_duration, double curr_time);

    double MinMaxBound(double value, double min, double max);
          
}

#endif
