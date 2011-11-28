#include <stdio.h>

double getMin(double a,double b,double c){
	
	double min = a;
	if(b<a || c<a){
		if(b<c) min=b;
		else if(c<b) min=c;
		else min=c;
	}
	return min;
	
}


void show(double dm[][4]){
	
	int dim = sizeof(dm[0])/sizeof(dm[0][0]);
	int i;
	int j;
	
	for(i=0;i<dim;i++){
		printf("\n");
		for(j=0;j<dim;j++){		
			printf(" %f ",dm[i][j]);
		}
		printf("\n");
	}
	printf("\n");
	return;

}

void getDistanceMap(int mat[][4], double dm[][4]){

	int dim = sizeof(mat[0])/sizeof(mat[0][0]);
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

}



int main(int argc, char **argv)
{
	int mat[4][4] = {{0,1,1,1},
					{1,1,1,0},
					{1,1,1,1},
					{1,0,0,1}};
	
		
	double dm[4][4];
	getDistanceMap(mat,dm);
	show(dm);
	
	return 0;
}






