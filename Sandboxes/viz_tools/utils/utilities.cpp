#include "utilities.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <Eigen/Eigenvalues>

//#include <Configuration.h>
#ifndef CTRL_CONFIG_H
#define THIS_COM "/home/orion/catkin_ws/src/dreamer/"
#endif

using namespace std;

namespace sejong{
    std::string pretty_string(sejong::Vector const & vv)
    {
        ostringstream os;
        pretty_print(vv, os, "", "", true);
        return os.str();
    }
  
    
    std::string pretty_string(sejong::Quaternion const & qq)
    {
        ostringstream os;
        pretty_print(qq, os, "", "", true);
        return os.str();
    }
    
    
    std::string pretty_string(sejong::Matrix const & mm, std::string const & prefix)
    {
        ostringstream os;
        pretty_print(mm, os, "", prefix);
        return os.str();
    }
    
    
    void pretty_print(sejong::Vector const & vv, std::ostream & os,
                      std::string const & title, std::string const & prefix,
                      bool nonl)
    {
        pretty_print((sejong::Matrix const &) vv, os, title, prefix, true, nonl);
    }
    
    
    void pretty_print(sejong::Quaternion const & qq, std::ostream & os,
                      std::string const & title, std::string const & prefix,
                      bool nonl)
    {
        pretty_print(qq.coeffs(), os, title, prefix, true, nonl);
    }
    
    
    std::string pretty_string(double vv)
    {
        static int const buflen(32);
        static char buf[buflen];
        memset(buf, 0, sizeof(buf));
/*#ifndef WIN32
        if (isinf(vv)) {
            snprintf(buf, buflen-1, " inf    ");
        }
        else if (isnan(vv)) {
            snprintf(buf, buflen-1, " nan    ");
        }
        else if (fabs(fmod(vv, 1)) < 1e-9) {
            snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(vv)));
        }
        else {
            snprintf(buf, buflen-1, "% 6.6f  ", vv);
        }
#else // WIN32*/
        snprintf(buf, buflen-1, "% 6.6f  ", vv);
//#endif // WIN32
        string str(buf);
        return str;
    }
  
    void pretty_print(const std::vector<double> & _vec, const char* title){
        printf("%s: ", title);
        for( int i(0); i< _vec.size(); ++i){
            printf("% 6.4f, \t", _vec[i]);
        }
        printf("\n");
    }
    void pretty_print(const double * _vec, const char* title, int size){
        printf("%s: ", title);
        for(int i(0); i< size; ++i){
            printf("% 6.4f, \t", _vec[i]);
        }
        printf("\n");
    }
    void pretty_print(sejong::Matrix const & mm, std::ostream & os,
                      std::string const & title, std::string const & prefix,
                      bool vecmode, bool nonl)
    {
        char const * nlornot("\n");
        if (nonl) {
            nlornot = "";
        }
        if ( ! title.empty()) {
            os << title << nlornot;
        }
        if ((mm.rows() <= 0) || (mm.cols() <= 0)) {
            os << prefix << " (empty)" << nlornot;
        }
        else {
            // if (mm.cols() == 1) {
            //   vecmode = true;
            // }
	
            if (vecmode) {
                if ( ! prefix.empty())
                    os << prefix;
                for (int ir(0); ir < mm.rows(); ++ir) {
                    os << pretty_string(mm.coeff(ir, 0));
                }
                os << nlornot;
	
            }
            else {

                for (int ir(0); ir < mm.rows(); ++ir) {
                    if ( ! prefix.empty())
                        os << prefix;
                    for (int ic(0); ic < mm.cols(); ++ic) {
                        os << pretty_string(mm.coeff(ir, ic));
                    }
                    os << nlornot;
                }
	  
            }
        }
    }

        void saveVector(const sejong::Vector & _vec, string _name){
        string file_name;
        cleaning_file(_name, file_name);

        std::ofstream savefile(file_name.c_str(), ios::app);
        for (int i(0); i < _vec.rows(); ++i){
            savefile<<_vec(i)<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }
    
    void saveVector(double * _vec, std::string _name, int size){
        string file_name;
        cleaning_file(_name, file_name);
        std::ofstream savefile(file_name.c_str(), ios::app);
        
        for (int i(0); i < size; ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }
    
    void saveVector(const std::vector<double> & _vec, string _name){
        string file_name;
        cleaning_file(_name, file_name);
        std::ofstream savefile(file_name.c_str(), ios::app);
        for (int i(0); i < _vec.size(); ++i){
            savefile<<_vec[i]<< "\t";
        }
        savefile<<"\n";
        savefile.flush();
    }

    void saveValue(double _value, string _name){
        string file_name;
        cleaning_file(_name, file_name);
        std::ofstream savefile(file_name.c_str(), ios::app);

        savefile<<_value <<"\n";
        savefile.flush();
    }

    void saveVectorSequence(const std::vector<sejong::Vector> & seq, string _name){
        string file_name;
        cleaning_file(_name, file_name);

        std::ofstream savefile(file_name.c_str(), ios::app);
        
        for (int i(0); i< seq.size(); ++i){
            for (int j(0); j< seq[i].rows(); ++j){
                savefile<<seq[i](j)<<"\t";
            }
            savefile<<"\n";
        }
        savefile.flush();
    }

    void printVectorSequence(const std::vector<sejong::Vector> & seq, string _name){
        std::cout<<_name<<":\n";
        for (int i(0); i< seq.size(); ++i){
            for (int j(0); j< seq[i].rows(); ++j){
                std::cout<<seq[i](j)<<"\t";
            }
            std::cout<<"\n";
        }
    }

    void cleaning_file(string  _file_name, string & _ret_file){
        _ret_file += THIS_COM"experiment_data/";
        _ret_file += _file_name;
        _ret_file += ".txt";
        
        std::list<string>::iterator iter = std::find(gs_fileName_string.begin(), gs_fileName_string.end(), _file_name);
        if(gs_fileName_string.end() == iter){
            gs_fileName_string.push_back(_file_name);
            remove(_ret_file.c_str());
        }
    }
    
    double generator_white_noise(double min, double var){

        static bool hasSpare = false;
        static double rand1, rand2;

        if(hasSpare){
            hasSpare = false;
            return sqrt(var*rand1)*sin(rand2) + min;
        }
        
        hasSpare = true;

        rand1 = rand()/ ((double)RAND_MAX);
        if(rand1 < 1e-100) rand1 = 1e-100;
        rand1 = -2*log(rand1);

        rand2 = (rand()/ ((double) RAND_MAX))* M_PI*2;

        return min + sqrt(var*rand1)*cos(rand2);
    }
    void sqrtm(const sejong::Matrix & mt, sejong::Matrix & sqrt_mt){
        Eigen::EigenSolver<sejong::Matrix> es(mt, true);
        Eigen::MatrixXcd V = es.eigenvectors();
        Eigen::VectorXcd Dv = es.eigenvalues();
        Eigen::MatrixXcd sqrtD(mt.cols(), mt.rows());
        sqrtD = Dv.cwiseSqrt().asDiagonal();
        Eigen::MatrixXcd tmp_sqrt_mt = V*sqrtD*V.inverse();

        sqrt_mt = tmp_sqrt_mt.real();
    }

    
    double smooth_changing(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = ini + (end - ini)*0.5*(1-cos(curr_time/moving_duration * M_PI));
        if(curr_time>moving_duration){
            ret = end;
        }
        return ret;
    }
    
    double smooth_changing_vel(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = (end - ini)*0.5*(M_PI/moving_duration)*sin(curr_time/moving_duration*M_PI);
        if(curr_time>moving_duration){
            ret = 0.0;
        }
        return ret;
    }
    double smooth_changing_acc(double ini, double end, double moving_duration, double curr_time){
        double ret;
        ret = (end - ini)*0.5*(M_PI/moving_duration)*(M_PI/moving_duration)*cos(curr_time/moving_duration*M_PI);
        if(curr_time>moving_duration){
            ret = 0.0;
        }
        return ret;
    }

    double MinMaxBound(double value, double min, double max){
        if(value > max){
            return max;
        }
        else if (value < min){
            return min;
        }
        return value;
    }
}
