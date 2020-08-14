#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <cmath>
#include <iostream>

//Variáveis para realizar o controle
geometry_msgs::Point velocidade;

float u,v=1,w,Kp = 10,tol=0.01,errox,erroy,R=0.08,L=0.16,x,y,theta;

//Função que vai receber os dados da posição do Coppelia e fazer o cálculo de W e V
void Dados_Posicao(const geometry_msgs::Point::ConstPtr& posicao_atual){
	if(v>tol){

	errox = x - posicao_atual->x;
	erroy = y - posicao_atual->y;
	
	//Obs: em z foi colocado o theta, já que a posição Z não é utilizada.
	theta = atan2(erroy,errox);
	
	u = theta - posicao_atual->z;

	w = Kp*u;
	
	v = sqrt(errox*errox + erroy*erroy);

	}
	else{
	v = 0;
	w=0;
}
	ROS_INFO("Posicao X: %.2f\nPosicao Y: %.2f\nTheta: %.2f\nU: .%2f\nW: .%2f\nV: %.2f\nx= %.2f\ny = %.2f",posicao_atual->x,posicao_atual->y,posicao_atual->z,u,w,v,x,y);

}

int main(int argc, char **argv){
	ros::init(argc,argv,"simple_cotrol");
	
	ros::NodeHandle n("~");

	ros::Subscriber sub_pos = n.subscribe("/dados_coppelia",1000,Dados_Posicao);

	ros::Publisher pub_vel = n.advertise<geometry_msgs::Point>("/comando1",1000);
	

	if ((n.getParam("X",x)) && (n.getParam("Y",y)))
		std::cout<<"Parametros X e Y lidos com sucesso, x = "<<x<<" y = "<<y;
	
	while(ros::ok()){	
		velocidade.x = (2*v-w*L)/(2*R); //Velocidade da Roda Esquerda
		velocidade.y = (2*v+w*L)/(2*R); //Velocidade da Roda Direita
		
		pub_vel.publish(velocidade);

	ros::spinOnce();
	}
	
return 0;
}
