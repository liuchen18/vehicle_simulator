#ifndef _POINT2D
#define _POINT2D


namespace collision{
    constexpr double kMathEpsilon = 1e-10;
    class point2d{
        public:
        point2d():x_(0),y_(0){};
        point2d(const double x,const double y):x_(x),y_(y){};
        double get_x() const{return x_;}
        double get_y() const{return y_;}
        double get_xsquare() const {return x_*x_;}
        double get_ysquare() const{return y_*y_;}
        void set_x(const double x){x_=x;}
        void set_y(const double y){y_=y;}
        void set_xy(const double x,const double y){x_=x;y_=y;}
        void set_xy(const point2d& another_point){x_=another_point.get_x();y_=another_point.get_y();}
        /*angle to the positive x semi axis*/
        double get_angle();

        void normalize();

        double get_length() const;

        /*compute distance to another given point*/
        double distance_to_point(const point2d& another) const;

        /*compute the cross product of the given point*/
        double crossproduct(const point2d& another) const;

        /*compute the inner product of the given point*/
        double innerproduct(const point2d& another) const;

        /*rewrite the operator to do normal operation*/
        point2d operator + (const point2d& another) const;
        point2d operator - (const point2d& another) const;
        point2d operator * (const double ratio) const;
        point2d operator / (const double ratio) const;
        point2d operator += (const point2d& another);
        point2d operator -= (const point2d& another);
        point2d operator *= (const double ratio);
        point2d operator /= (const double ratio);
        bool operator == (const point2d& another) const;



        private:
        double x_,y_;
    };
    point2d operator * (const double ratio,const point2d& point);


}

#endif