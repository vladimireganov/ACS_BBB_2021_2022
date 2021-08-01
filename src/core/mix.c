/**
 * @file mixing_matrix.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <float.h> // for DBL_MAX
#include <mix.h>

 /**
  * 4-rotor X layout for a rocket
  * top view:
  *  4   1       cw cw       Z   X up
  *    X                     ^
  *  3   2       cw cw       + > Y
  *
  * columns: X Y Z Roll Pitch Yaw
  * rows: motors 1-4
  */
static double mix_4x[][6] = { \
{-1.0,   0.0,  0.0,   0.0,  -0.5,   0.5},\
{-1.0,   0.0,  0.0,   0.0,   0.5,   0.5},\
{-1.0,   0.0,  0.0,   0.0,   0.5,  -0.5},\
{-1.0,   0.0,  0.0,   0.0,  -0.5,  -0.5} };




/**
 * 4-rotor + layout for a rocket
 *
 * top view:
 *    1		  cw     	Z   X up 
 *  4 + 2   cw  cw		^
 *    3       cw      	+ > Y
 * columns: X Y Z Roll Pitch Yaw
 * rows: actuators 1-4
 */
static double mix_4plus[][6] = { \
{-1.0,   0.0,  0.0,   0.0,   0.5,   0.0},\
{-1.0,   0.0,  0.0,   0.0,   0.0,  -0.5},\
{-1.0,   0.0,  0.0,   0.0,  -0.5,   0.0},\
{-1.0,   0.0,  0.0,   0.0,   0.0,   0.5}};


static double (*mix_matrix)[6];
static int initialized;
static int rotors;
static int dof;


int mix_init(rotor_layout_t layout)
{
	switch(layout){
	case LAYOUT_4X:
		rotors = 4;
		dof = 4;
		mix_matrix = mix_4x;
		break;
	case LAYOUT_4PLUS:
		rotors = 4;
		dof = 6;
		mix_matrix = mix_4plus;
		break;
	default:
		fprintf(stderr,"ERROR in mix_init() unknown rotor layout\n");
		return -1;
	}

	initialized = 1;
	return 0;
}


int mix_all_controls(double u[6], double* mot)
{
	int i,j;
	if(initialized!=1){
		fprintf(stderr,"ERROR in mix_all_controls, mixing matrix not set yet\n");
		return -1;
	}
	// sum control inputs
	for(i=0;i<rotors;i++){
		mot[i]=0.0;
		for(j=0;j<6;j++){
			mot[i]+=mix_matrix[i][j]*u[j];
		}
	}
	// ensure saturation, should not need to do this if mix_check_saturation
	// was used properly, but here for safety anyway.
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}


int mix_check_saturation(int ch, double* mot, double* min, double* max)
{
	int i, min_ch;
	double tmp;
	double new_max = DBL_MAX;
	double new_min = -DBL_MAX;

	if(initialized!=1){
		fprintf(stderr,"ERROR: in check_channel_saturation, mix matrix not set yet\n");
		return -1;
	}

	switch(dof){
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr,"ERROR: in check_channel_saturation, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if(ch<min_ch || ch>=6){
		fprintf(stderr,"ERROR: in check_channel_saturation, ch out of bounds\n");
		return -1;
	}

	// make sure motors are not already saturated
	for(i=0;i<rotors;i++){
		if(mot[i]>1.0 || mot[i]<0.0){
			fprintf(stderr,"ERROR: motor channel already out of bounds\n");
			return -1;
		}
		/*if (mot[i] == 0.0) {
			//printf("\n mot[i] == 0.0, i = %d\n", i);
			if (mix_matrix[i][ch] == 0.0) continue;
			
			
			// for positive entry in mix matrix
			if (mix_matrix[i][ch] > 0.0)	tmp = (1.0) / mix_matrix[i][ch];
			// for negative entry in mix matrix
			else tmp = -1.0 / mix_matrix[i][ch];
			// set new upper limit if lower than current
			if (tmp < new_max) new_max = tmp;
		}*/
	}

	

	// find max positive input
	for(i=0;i<rotors;i++){
		//printf("\n i = %d, Ch =%d  mix_matrix[i][ch] = %f and mot[i] = %f\n", i, ch, mix_matrix[i][ch],mot[i]);

		// if mix channel is 0, impossible to saturate
		if(mix_matrix[i][ch]==0.0) continue;
		// for positive entry in mix matrix
		if(mix_matrix[i][ch]>0.0)	tmp = (1.0-mot[i])/mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = -mot[i]/mix_matrix[i][ch];
		// set new upper limit if lower than current
		if(tmp<new_max) new_max = tmp;
	}
	//printf(" new_max = %f ", new_max);

	// find min (most negative) input
	for(i=0;i<rotors;i++){
		// if mix channel is 0, impossible to saturate
		if(mix_matrix[i][ch]==0.0) continue;
		// for positive entry in mix matrix
		if(mix_matrix[i][ch]>0.0)	tmp = -mot[i]/mix_matrix[i][ch];
		// for negative entry in mix matrix
		else tmp = (1.0-mot[i])/mix_matrix[i][ch];
		// set new upper limit if lower than current
		if(tmp>new_min) new_min = tmp;
	}
	//printf(" new_min = %f \n", new_min);

	*min = new_min;
	*max = new_max;
	return 0;
}


int mix_add_input(double u, int ch, double* mot)
{
	int i;
	int min_ch;

	if(initialized!=1 || dof==0){
		fprintf(stderr,"ERROR: in mix_add_input, mix matrix not set yet\n");
		return -1;
	}
	switch(dof){
	case 4:
		min_ch = 2;
		break;
	case 6:
		min_ch = 0;
		break;
	default:
		fprintf(stderr,"ERROR: in mix_add_input, dof should be 4 or 6, currently %d\n", dof);
		return -1;
	}

	if(ch<min_ch || ch>=6){
		fprintf(stderr,"ERROR: in mix_add_input, ch out of bounds\n");
		return -1;
	}

	// add inputs
	for(i=0;i<rotors;i++){
		mot[i] += u*mix_matrix[i][ch];
		// ensure saturation, should not need to do this if mix_check_saturation
		// was used properly, but here for safety anyway.
		if(mot[i]>1.0) mot[i]=1.0;
		else if(mot[i]<0.0) mot[i]=0.0;
	}
	return 0;
}


