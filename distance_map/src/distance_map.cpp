#include <stdio.h>
#include <ros/ros.h>
#include <occupancy_map/OccupancyMap.h>
#include <distance_map/DistanceMap.h>

static ros::Publisher distance;

double getMin(double a,double b,double c){
	
	double min = a;
	if(b<a || c<a){
		if(b<c) min=b;
		else if(c<b) min=c;
		else min=c;
	}
	return min;
	
}

void obstacleCallback(const occupancy_map::OccupancyMap& msg)
{
	int mat[msg.size_x][msg.size_y];
	double dm[msg.size_x][msg.size_y];
	for (int i = 0; i < msg.size_y; i++) {
		for (int j = 0; j < msg.size_x; j++) {
			if (msg.map[i*msg.size_x+j])
				mat[i][j] = 0;
			else
				mat[i][j] = 1;
		}
	}
	
	int dim = msg.size_x;
	int i,x,y;
	double MAX = 1000;
	
	//first 
	for(i=0;i<dim;i++){
		
		for(x=i,y=0; x >= 0; x--,y++){
			
				if(mat[x][y] == 0) dm[x][y]=0;
				
				else if(x>0 && y>0){
					
					if(mat[x-1][y] == 0 || mat[x-1][y-1] == 0 || mat[x][y-1] == 0 || mat[x+1][y-1] == 0 || mat[x-1][y+1] == 0 ){
						double a = getMin(mat[x-1][y]+1,mat[x-1][y-1]+1.41,mat[x][y-1]+1);
						double b = getMin(a,mat[x+1][y-1]+1.41,mat[x-1][y+1]+1.41);
							if(a<=b) dm[x][y]=a;
							else dm[x][y]=b;
					}
					else{
						double c = getMin(dm[x+1][y-1]+1.41, dm[x][y-1]+1, dm[x+1][y-1]+1.41);
						double d = dm[x-1][y]+1;
							if(c<=d) dm[x][y]=c;
							else dm[x][y]=d;							
					}	
				}
				
				else{ 
				
					if(x==0 && y==0) dm[x][y]=MAX;
					
					else if(x == 0){
						if(mat[x][y-1]==0) dm[x][y]=1;
						else if(mat[x+1][y-1]==0) dm[x][y]=1.41;
						else{
							if(dm[x][y-1]<=dm[x+1][y-1]) dm[x][y] = dm[x][y-1]+1;
							else dm[x][y] = dm[x+1][y-1]+1.41;
						}
					}
					
					else if(y == 0){
						if(mat[x-1][y]==0)dm[x][y]=1;
						else if(mat[x-1][y+1]==0)dm[x][y]=1.41;
						else dm[x][y] = dm[x-1][y]+1;
					}
					
				}
		}
	}

	//second
	for(i=1;i<dim;i++){
		
		for(x=dim-1,y=i; y < dim; x--,y++){
				
				if(mat[x][y]==0)dm[x][y]=0;
				
				else if(x-1>=0 && y-1>=0){
					
					if(mat[x-1][y] == 0 || mat[x-1][y-1] == 0 || mat[x][y-1] == 0)
						dm[x][y] = getMin(mat[x-1][y]+1,mat[x-1][y-1]+1.41,mat[x][y-1]+1);
				
					else
						dm[x][y] = getMin(dm[x-1][y]+1, dm[x-1][y-1]+1.41, dm[x][y-1]+1);
						
				}
				
				else{
					
					
					if(x+1>=dim){
						if(mat[x][y-1]==0 || mat[x+1][y]==0) dm[x][y]=1;
						else if(mat[x+1][y-1]==0 || mat[x-1][y+1]==0) dm[x][y]=1.41;
						else dm[x][y] = getMin(dm[x][y-1]+1,dm[x-1][y-1]+1.41,dm[x-1][y]+1);
						
					}
					
					else if(y+1>=dim){
						if(mat[x][y-1]==0 || mat[x+1][y]==0) dm[x][y]=1;
						else if(mat[x+1][y-1]==0 || mat[x-1][y-1]==0) dm[x][y]=1.41;
						else{ 
							double a = getMin(dm[x][y-1]+1,dm[x+1][y-1]+1.41,dm[x+1][y]+1);
							double b = mat[x-1][y-1];
							if(a<=b) dm[x][y]=a;
							else dm[x][y]=b;
						}					
					
					}
				}
		}
	}		
	
	//third
	for(i=2;i<=dim;i++){
		for(x=dim-1,y=dim-i; y<dim; y++,x--){
				
				if(x+1<dim && y+1<dim){
					
						double a = getMin(dm[x+1][y-1]+1.41,dm[x+1][y]+1,dm[x+1][y+1]+1.41);
						double b = getMin(a,dm[x][y+1]+1,dm[x-1][y+1]+1.41);
						dm[x][y] = getMin(a,b,dm[x][y]);
					

				}
				else{
						if(x+1>=dim){
							dm[x][y] = getMin(dm[x][y+1]+1,dm[x-1][y+1]+1.41,dm[x][y]);
						}
						
						if(y+1>=dim){
							dm[x][y] = getMin(dm[x+1][y]+1,dm[x+1][y-1]+1.41,dm[x][y]);
						}	
		
				}	
				
				
		}
	}



	//fourth

	for(i=1;i<dim;i++){
		for(x=dim-1-i,y=0; x >= 0; y++,x--){
				
				if(x>0 && y>0){
					
						double a = getMin(dm[x+1][y-1]+1.41,dm[x+1][y]+1,dm[x+1][y+1]+1.41);
						double b = getMin(a,dm[x][y+1]+1,dm[x-1][y+1]+1.41);
						dm[x][y] = getMin(a,b,dm[x][y]);
					

				}
				else{
						
						if(x==0 && y==0){
							 if(mat[x][y] == 0) dm[x][y]=0;
							 else dm[x][y]=getMin(dm[x][y+1]+1,dm[x+1][y+1]+1.41,dm[x+1][y]+1);
						}
												
						else if(x<=0){
							double c = getMin(dm[x+1][y-1]+1.41, dm[x+1][y]+1, dm[x+1][y+1]+1.41);
							dm[x][y] = getMin(dm[x][y],c,dm[x][y+1]+1);
						}
						
						else if(y<=0){
							double c = getMin(dm[x-1][y+1]+1.41, dm[x][y+1]+1, dm[x+1][y+1]+1.41);
							dm[x][y] = getMin(dm[x][y],c,dm[x+1][y]+1);
						}	
		
				}	
				
				
		}
	}
	
	distance_map::DistanceMap msg_o;
	msg_o.size_x = msg.size_x;
	msg_o.size_y = msg.size_y;
	std::vector<double> matrix(msg.size_x*msg.size_y);
	for (int j = 0; j < msg.size_y; j++)
		for (int i = 0; i < msg.size_x; i++)
			matrix[j*msg.size_x+i] = dm[i][j];
	msg_o.map = matrix;
	distance.publish(msg_o);
	
	/*
	for(int i=0;i<dim;i++){
		printf("\n");
		for(int j=0;j<dim;j++){		
			printf(" %2.1f ",dm[i][j]);
		}
		printf("\n");
	}
	printf("\n");
	*/
	
	ros::Rate loop_rate(2);
	loop_rate.sleep();

}




int main(int argc, char **argv)
{
	/*
	int mat[4][4] = {{0,1,1,1},
					{1,1,1,0},
					{1,1,1,1},
					{1,0,0,1}};
	double dm[4][4];
	getDistanceMap(mat,dm);
	show(dm);
	*/
	ros::init(argc, argv, "distance_map_generator");
	ros::NodeHandle n;
	ros::Subscriber obstacle = n.subscribe("obstacle_map", 1, obstacleCallback);
	distance = n.advertise<distance_map::DistanceMap>("distance_map", 1);
	
	ros::spin();
	
	return 0;
}






