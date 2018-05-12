#ifndef PID_H
#define PID_H

class pid_t
{
	public:
		pid_t(const float p_gain,const float i_gain,const float d_gain):target(0.0),smoothing(0.0),
			limit(0.0),d_smooth_m(0.0),error_old_m(0.0),error_total_m(0.0)
		{
			gains[0]=p_gain;
			gains[1]=i_gain;
			gains[2]=d_gain;
		}

		float update(const float value)
		{
			float error[3]={0,0,0};

			error[0]=target-value;
			error[1]=error_total_m;
			error[2]=error[0]-error_old_m;

			d_smooth_m=error[2]*smoothing+d_smooth_m*(1.0-smoothing);

			error_old_m=error[0];

			if(error_total_m>limit)
				error_total_m=limit;
			if(error_total_m<-limit)
				error_total_m=-limit;

			error_total_m+=error[0];

			return gains[P]*error[0]+gains[1]*error[1]+gains[2]*d_smooth_m;
		}

		void reset()
		{
			error_total_m=0;
		}

		float gains[3];
		float target;
		float smoothing;
		float limit;

	private:
		float d_smooth_m;
		float error_old_m;
		float error_total_m;
};

#endif