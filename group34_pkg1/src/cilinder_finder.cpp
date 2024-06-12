#include "cilinder_finder.h"

double dist(int k, double d0, double delta, double r) {
    double half, angle, a, h, dk;
    half = atan2(r, d0);
    if (k*delta <= half) angle = k*delta;
    else if (k*delta <= 2*half) angle = 2*half-k*delta;
    else return -1;
    a = r - d0*tan(angle);
    h = 0.5*(-2*a*sin(angle)+sqrt((2*a*sin(angle))*(2*a*sin(angle))-4*((a+r)*(a-r))));
    dk = -h+d0/cos(angle);
    return dk;
} 

vector<double> center_cart(double d0, double theta0, double r) {
    double half, dc, xc, yc;
    half = atan2(r,d0);
    dc = d0/cos(half);
    xc = dc*cos(theta0+half) + LASER_OFFSET_X;
    yc = dc*sin(theta0+half); 
    vector<double> center{xc, yc};
    return center;
}

vector<double> center_polar(double d0, double theta0, double r) {
    double half, dc, thetac;
    half = atan2(r,d0);
    dc = d0/cos(half);
    thetac = (theta0 + half)*180/PI;
    vector<double> center{dc, thetac};
    return center;
}

vector<vector<double>> cilinder_wf(vector<vector<double>> cilinders, vector<double> tiago_pose) {
    vector<vector<double>> cilinder_wf;
    double xt = tiago_pose[0];
    double yt = tiago_pose[1];
    double angle = tiago_pose[2];
    for (int i = 0; i < cilinders.size(); i++) {
        double xc = cilinders[i][0];
        double yc = cilinders[i][1];
        double xc_w = xt + xc*cos(angle) - yc*sin(angle);
        double yc_w = yt + xc*sin(angle) + yc*cos(angle);
        cilinder_wf.push_back({xc_w, yc_w});
    }
    return cilinder_wf;
}

vector<vector<double>> cilinder_scan(vector<double> laser, vector<double> tiago_pose, double delta, double r, double laser_angle_min) {
	if (laser.size() == 0) {
		return {{-1}};
	}
	if (tiago_pose.size() != 3) {
		return {{-1}};
	}
    double threshold = 0.5;
    double error = 0.1;
    vector<vector<double>> possible_cilinder;
    for (int i = 0; i < laser.size(); i++) {
        double d0, theta0;
        if (laser[i+1] < 0.2) continue;
        if (abs(laser[i]-laser[i+1]) > threshold) {
            bool is_cilinder = false;
            if (laser[i+1]<laser[i]) {
                d0=laser[i+1];
                theta0 = (i+1)*delta+laser_angle_min;
                for (int k = 1; k <= 20; k++) {
                    if ((i+1)+k > laser.size()) {
                        break;
                    }
                    double dk = dist(k, d0, delta, r);
                    if (laser[(i+1)+k] <= (dk+error) && laser[k+(i+1)] >= (dk-error)){
                        is_cilinder = true;
                    } else {
                        is_cilinder = false;
                        break;
                    }
                }
            }
            /*
            else {
                d0=laser[i];
                theta0 = i*delta+LASER_ANGLE_MIN;
                for (int k = 1; k <= 10; k++) {
                    if (i-k < 0) {
                        break;
                    }
                    double dk = dist(k, d0, delta, r);
                    if (laser[i-k] <= (dk+error) && laser[i-k] >= (dk-error)){
                        is_cilinder = true;
                    }
                }
            }
            */
            if (is_cilinder) {
                possible_cilinder.push_back(center_cart(d0, theta0, r));
            }      
        }
    }
    
    vector<vector<double>> cilinders_wf;
    cilinders_wf = cilinder_wf(possible_cilinder, tiago_pose);
	
	if (possible_cilinder.size() > 1) {
		bool not_sorted = true;
    	while (not_sorted) {
        	not_sorted = false;
        	for (int i = 0; i < cilinders_wf.size()-1; i++) {
            	if (cilinders_wf[i][1] < cilinders_wf[i+1][1]) {
                	vector<double> aux;
                	aux = cilinders_wf[i];
                	cilinders_wf[i] = cilinders_wf[i+1];
                	cilinders_wf[i+1] = aux;
                	not_sorted = true;
            	}
        	}
    	}
    } 

    return cilinders_wf;
}

