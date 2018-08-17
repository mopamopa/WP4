#include <time.h> // for srand(time(NULL));
# include <stdio.h>
#include <iostream>
//#include <string>
# include <math.h>
#include <list>
using namespace std;

//Utilita'
double Segno(double numero);
double TrovaMinimo(double *Array, int MaxDim);
int TrovaIndiceDelMinimo(double *Array, int MaxDim);
int TrovaIndiceDelMassimo(double *Array, int MaxDim);
double TrovaMassimo(double *Array, int MaxDim);
double NumeroRand(double max);

////////////////////////////
struct Pkt{
	int car;   
	double v;
	double d;	
};

enum event {Schedula_dinamica, sampling, Ricezione_Update};
struct track
{
  double istant;
  event evento;
  int car;
  
  Pkt pkt;

  public:
  bool operator < (track); 

};
list<track>L(0);
////////////////////////////

//////////////////////////////////////////////////////
// ToVT = L. Xu, L. Y. Wang, G. Yin and H. Zhang, "Communication Information Structures and Contents for Enhanced Safety of Highway Vehicle Platoons," ...
// ... in IEEE Transactions on Vehicular Technology, vol. 63, no. 9, pp. 4206-4220, Nov. 2014. doi: 10.1109/TVT.2014.2311384
int NumCar, memo, memo2; 
double stepsize, dref, vref, mref, *d, *v, *m, *b, *delay, *v_m, *d_m, F0, stepsize2, **Id, **Iv;
int collision; bool stampaoutput=0;

void SchedulaDinamica(double ClockAttuale);
void SchedulaDinamica2(double ClockCurrent, int car);
void SchedulaDinamica3(double ClockCurrent, int car);
double g1(double d); double g1dshape(double d);
void Fsampling(double ClockCurrent);
void RicezioneUpdate(double ClockCurrent, Pkt p);

//////////////////////////////////////////////////////

int ApplyRules(int F0, int m, int d_ms, int d0, int v0);

bool Enable_ApplyRules=1;

int ApplyRules(int F0, int m, int d_ms, int d0, int v0) {
  if ((F0 > 1672) && (v0 <= 39)) return 0;
  if ((v0 <= 30)) return 0;
  if ((d0 > 31) && (v0 <= 70)) return 0;
  if ((F0 > 2484) && (d0 > 24 && d0 <= 33) && (v0 <= 88)) return 0;
  if ((m <= 1850) && (d_ms > 10 && d_ms <= 61) && (d0 > 22)) return 0;
  if ((m <= 1707) && (d0 > 18) && (v0 > 36 && v0 <= 50)) return 0;
  if ((F0 > 1610) && (m <= 1149) && (d_ms > 65 && d_ms <= 192) && (d0 > 21) && (v0 > 51 && v0 <= 86)) return 0;
  if ((d_ms > 47 && d_ms <= 48)) return 0;
  if ((F0 <= 4301) && (d0 <= 24) && (v0 > 34)) return 1;
  if ((d0 <= 29) && (v0 > 62)) return 1;
  if ((v0 > 77)) return 1;
  if ((F0 <= 4113) && (m > 1664) && (d_ms > 14) && (v0 > 27)) return 1;
  if ((F0 <= 2547) && (m > 1248 && m <= 1815) && (d_ms > 65)) return 1;
  if ((F0 > 4253) && (v0 > 56)) return 1;

  return 0;
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void main(){	

	FILE *Output2; Output2=fopen("LogMetrics.txt","w"); fclose(Output2);
	Output2=fopen("LogMetrics.txt","w"); fprintf(Output2,"N\tF0\tm\td_ms\td0\tv0\tcollision"); fprintf(Output2,"\n"); fclose(Output2);

	srand(time(NULL));

	int ii, delay_ms, maxx=100; printf("\n\n"); stampaoutput=0; 
	for(ii=0;ii<maxx;ii++){

		printf("%d of %d\n",ii,maxx); if(ii==maxx-1) stampaoutput=1; 

		NumCar=3;
		
		F0 = NumeroRand( (3e3-100) )+100+1500; F0*=-1; 
		mref=500+NumeroRand( (2e3-500) );		
		delay_ms=(int)NumeroRand( (200-10) )+10;		
		dref=NumeroRand((40-15))+15;
		vref=NumeroRand((90-10))+10; // km/h

		/*F0=-4000;
		mref=1500;		
		delay_ms=25; 
		vref=70; // 50 safe 90 unsafe  
		dref=25; // con v=70 30 safe 10 unsafe*/
		
		Output2=fopen("LogMetrics.txt","a"); 
		fprintf(Output2,"%d\t%d\t%d\t%d\t%d\t%d",NumCar,-(int)F0,(int)mref,delay_ms,(int)dref,(int)vref); 
		fclose(Output2);
		
		collision=0;

		/////////////////////////////////////////////////
		// import of rules
		if(Enable_ApplyRules){
			int predicted_collision=ApplyRules(F0, mref, delay_ms, dref, vref); 				

			if(predicted_collision){
				printf("\tDANGER predicted collision with: force %d[N] delay %d[ms] distance %d[m] speed %d[Km/h]\n\n",-(int)F0,delay_ms,(int)dref,(int)vref); 				
			}else	 
				printf("\tSAFE no collision with: force %d[N] delay %d[ms] distance %d[m] speed %d[Km/h]\n\n",-(int)F0,delay_ms,(int)dref,(int)vref); 
			system("pause");
		    //printf("\npress a key to continue..."); scanf("%s");
		}		
		/////////////////////////////////////////////////

int i,p; event EventoCurrent; bool exit=false, nodelay;  
vref=vref*1000/3600; // m/s da Km/h

////////////////
FILE *Output; Output=fopen("O.txt","w"); fclose(Output);
bool stampagrandezze=false;
if(stampagrandezze){
Output=fopen("O.txt","w"); 
fprintf(Output,"t"); 
for(i=0;i<NumCar;i++) fprintf(Output,"\tv%d",i); 
for(i=1;i<NumCar;i++) fprintf(Output,"\td%d",i); 
fprintf(Output,"\n");	
fclose(Output); }
////////////////

double DurataSimulazione=1000, clock=0, sum_v=100; 
stepsize=(double)(delay_ms/4)/1000; stepsize2=stepsize*1;

d=new double [NumCar]; for(i=0;i<NumCar;i++) d[i]=dref; // m pag. 4 ToVT
v=new double [NumCar]; for(i=0;i<NumCar;i++) v[i]=vref; // m/s pag. 4 ToVT
m=new double [NumCar]; for(i=0;i<NumCar;i++) m[i]=mref; // Kg pag. 3 ToVT
// a: tire/road rolling resistance, a=0 pag. 3 ToVT 
// b: aerodynamic drag coeffcient pag. 2 ToVT 
b=new double [NumCar]; for(i=0;i<NumCar;i++) b[i]=0.43; // Kg/m pag. 3 ToVT 
delay=new double [NumCar]; for(i=0;i<NumCar;i++) delay[i]=(double)delay_ms/1000; 
v_m=new double [NumCar]; for(i=0;i<NumCar;i++) v_m[i]=v[i]; 
d_m=new double [NumCar]; for(i=0;i<NumCar;i++) d_m[i]=d[i]; 

///////////////////////////
/*memo=delay[0]/stepsize+1;
memo2=memo/2;
Id=new double *[memo];
for(p=0;p<memo;p++) Id[p]=new double[NumCar];
for(i=0;i<memo;i++) for(p=0;p<NumCar;p++) Id[i][p]=d[p]; 
//for(i=0;i<memo;i++) for(p=0;p<NumCar;p++) Id[i][p]=p;  
printf("\nId=["); 
for(i=0;i<memo;i++){
	printf("\n"); 
	for(p=0;p<NumCar;p++) 
		printf("Id[%d][\%d]=%.1f ",i,p,Id[i][p]);  
}
printf("\n];\n");

Iv=new double *[memo];
for(p=0;p<memo;p++) Iv[p]=new double[NumCar];
for(i=0;i<memo;i++) for(p=0;p<NumCar;p++) Iv[i][p]=v[p]; 
printf("\nIv=["); 
for(i=0;i<memo;i++){
	printf("\n"); 
	for(p=0;p<NumCar;p++) 
		printf("Iv[%d][\%d]=%.1f ",i,p,Iv[i][p]);  
}
printf("\n];\n");*/
//return;
///////////////////////////

track TrackCurrent; nodelay=false;
if(nodelay) { TrackCurrent.istant = stepsize; TrackCurrent.evento=Schedula_dinamica; L.push_back(TrackCurrent); }
else{
	for(i=0;i<NumCar;i++){ TrackCurrent.istant = stepsize; TrackCurrent.evento=Schedula_dinamica; TrackCurrent.car=i; L.push_back(TrackCurrent); }	
	for(i=0;i<NumCar-1;i++){ 
		TrackCurrent.car=i; // car=0
		TrackCurrent.istant = delay[i+1]; // delay[1]
		TrackCurrent.pkt.car=i; // i=0
		TrackCurrent.pkt.v=v[i]; // v[0]
		TrackCurrent.pkt.d=d[i+1]; // d[1]
		TrackCurrent.evento=Ricezione_Update;  // sottointeso per il veicolo 1
		L.push_back(TrackCurrent); 
	}	
}
TrackCurrent.istant = stepsize2; TrackCurrent.evento=sampling; TrackCurrent.car=-1; L.push_back(TrackCurrent);
L.sort();

while( clock<=DurataSimulazione && !exit){

        EventoCurrent=L.front().evento;
        clock=L.front().istant;

        switch(EventoCurrent)
        {
				case Schedula_dinamica: if(nodelay) SchedulaDinamica(clock); else SchedulaDinamica3(clock,L.front().car); break;			
				case sampling: Fsampling(clock);break;               				
				case Ricezione_Update: RicezioneUpdate(clock, L.front().pkt);break;
        }

        L.pop_front(); 
		
		sum_v=v[0]; for(i=1;i<NumCar;i++) sum_v+=v[i];
		if(sum_v==0) { exit=true; Fsampling(clock); }
        
}//END while(clock<=DurataSimulazione)

	Output2=fopen("LogMetrics.txt","a"); 
	fprintf(Output2,"\t%d",collision); 
	fprintf(Output2,"\n");	fclose(Output2);

////////////////
if(stampagrandezze){
Output=fopen("O.txt","a"); 
fprintf(Output,"t"); 
for(i=0;i<NumCar;i++) fprintf(Output,"\tv%d",i); 
for(i=1;i<NumCar;i++) fprintf(Output,"\td%d",i); 
fprintf(Output,"\n");	
fclose(Output); }
////////////////

delete d,m,v,b,delay,v_m,d_m;
//for(i=0;i<memo;i++) delete( Id[i] ); delete Id;
//for(i=0;i<memo;i++) delete( Iv[i] ); delete Iv;

}// end ciclo for() on LogMetrics.txt

printf("\n\n"); 
} // end main 
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void SchedulaDinamica(double ClockAttuale){

	int i;
	v[0]=v[0] + (stepsize/(m[0]))*(F0-b[0]*pow(v[0],2));
	
	//for(i=1;i<NumCar;i++) v[i]=v[i] + (stepsize/(m[i]))*(g1(d[i])-b[i]*pow(v[i],2));
	i=1; v[i]=v[i] + (stepsize/(m[i]))*(g1(d[i])-b[i]*pow(v[i],2));
	i=2; v[i]=v[i] + (stepsize/(m[i]))*(0.5*g1(d[i-1])+0.5*g1(d[i])-b[i]*pow(v[i],2));	
		
	for(i=0;i<NumCar;i++) if(v[i]<0) v[i]=0;

	for(i=1;i<NumCar;i++) 
		d[i]=d[i]+stepsize*(v[i-1]-v[i]);
	
	for(i=1;i<NumCar;i++) if(d[i]<0) d[i]=0;
		
	track TrackCurrent;
	TrackCurrent.istant = ClockAttuale+stepsize; TrackCurrent.evento=Schedula_dinamica; L.push_back(TrackCurrent); 
	L.sort();

}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void SchedulaDinamica3(double ClockCurrent, int car){

	int i,p;
	if(car==0){
		v[car]=v[car] + (stepsize/(m[car]))*(F0-b[car]*pow(v[car],2)); 
		if(v[car]<0) v[car]=0;	
	}else{				

		v[car]=v[car] + (stepsize/(m[car]))*( g1(d_m[car]) -b[car]*pow(v[car],2));	
		/*if(car==1) v[car]=v[car] + (stepsize/(m[car]))*( g1(d_m[car]) -b[car]*pow(v[car],2));
		if(car==2) v[car]=v[car] + (stepsize/(m[car]))*(0.5*g1( d_m[car-1])+0.5*g1(d_m[car]) -b[car]*pow(v[car],2));	*/
		if(v[car]<0) v[car]=0;		
		
		d[car]=d[car]+stepsize*(v_m[car-1]-v[car]);				
		//d[car]=d[car]+stepsize*(v[car-1]-v[car]);				
		if(d[car]<0) d[car]=0;	
		if(d[car]<5) collision=1;
	}	
					
	track TrackCurrent;	
	TrackCurrent.istant = ClockCurrent+stepsize; TrackCurrent.evento=Schedula_dinamica; TrackCurrent.car=car; L.push_back(TrackCurrent); 
	L.sort();
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void SchedulaDinamica2(double ClockCurrent, int car){

	int i,p;
	if(car==0){
		v[car]=v[car] + (stepsize/(m[car]))*(F0-b[car]*pow(v[car],2)); 
		if(v[car]<0) v[car]=0;
		for(i=0;i<memo-1;i++)	 Iv[i+1][car]=Iv[i][car]; Iv[0][car]=v[car];
	}else{				
		
		//v[car]=v[car] + (stepsize/(m[car]))*(g1(d[car])-b[car]*pow(v[car],2));
		/*if(car==1) v[car]=v[car] + (stepsize/(m[car]))*(g1(d[car])-b[car]*pow(v[car],2));
		if(car==2) v[car]=v[car] + (stepsize/(m[car]))*(0.5*g1(d[car-1])+0.5*g1(d[car])-b[car]*pow(v[car],2));*/
		
		//if(car==1) v[car]=v[car] + (stepsize/(m[car]))*(g1(d[car])-b[car]*pow(v[car],2));		
		if(car==1) v[car]=v[car] + (stepsize/(m[car]))*( g1(Id[memo2-1][car]) -b[car]*pow(v[car],2));
		if(car==2) v[car]=v[car] + (stepsize/(m[car]))*(0.5*g1( Id[memo-1][car-1])+0.5*g1(Id[memo-1][car]) -b[car]*pow(v[car],2));	

		if(v[car]<0) v[car]=0;

		/*printf("\nId=["); 
		for(i=0;i<memo;i++){
			printf("\n"); 
			for(p=0;p<NumCar;p++) 
				printf("Id[%d][\%d]=%.1f ",i,p,Id[i][p]);  
		}
		printf("\n];\n");*/		
		
		/*double gs1=g1(d[1]); double gs2=g1(d[2]);
		double gs1_d=g1(Id[memo-1][1]); double gs2_d=g1(Id[memo-1][2]);*/
		
		d[car]=d[car]+stepsize*(v[car-1]-v[car]);		
		//d[car]=Id[memo-1][car]+stepsize*(v[car-1]-v[car]);	
		/*if(car==2)	d[car]=d[car]+stepsize*(Iv[memo-1][car-1]-v[car]);
		if(car==1)	d[car]=d[car]+stepsize*(Iv[memo2-1][car-1]-v[car]);*/		

		if(d[car]<0) d[car]=0;

		for(i=0;i<memo-1;i++)	 Id[i+1][car]=Id[i][car]; Id[0][car]=d[car];
		for(i=0;i<memo-1;i++)	 Iv[i+1][car]=Iv[i][car]; Iv[0][car]=v[car];
	}	
					
	track TrackCurrent;	
	TrackCurrent.istant = ClockCurrent+stepsize; TrackCurrent.evento=Schedula_dinamica; TrackCurrent.car=car; L.push_back(TrackCurrent); 
	L.sort();
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void RicezioneUpdate(double ClockCurrent, Pkt packet){

	// packet.car=0, RicezioneUpdate() sottointeso per il veicolo 1
	d_m[packet.car+1]=packet.d; // d[1]
	v_m[packet.car]=packet.v; // v[0]

	track TrackCurrent;	
	TrackCurrent.car=packet.car; // 0
	TrackCurrent.istant = ClockCurrent+delay[packet.car+1]; // clock+delay[1]	

	TrackCurrent.pkt.car=packet.car; // 0
	TrackCurrent.pkt.v=v[packet.car]; // v[0]
	TrackCurrent.pkt.d=d[packet.car+1]; // d[1]

	TrackCurrent.evento=Ricezione_Update;  
	L.push_back(TrackCurrent); 
	L.sort();
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
void Fsampling(double ClockCurrent){

	int i;
	if(stampaoutput){
		FILE *Output=fopen("O.txt","a"); 
		fprintf(Output,"%.2f",ClockCurrent); 
		for(i=0;i<NumCar;i++) fprintf(Output,"\t%.6f",v[i]*3600/1000); 
		for(i=1;i<NumCar;i++) fprintf(Output,"\t%.2f",d[i]); 	
		fprintf(Output,"\n");	
		fclose(Output);
	}

	//for(i=0;i<NumCar;i++) if(d[i]<5) collision=1;

	track TrackCurrent;
	TrackCurrent.istant = ClockCurrent+stepsize2; TrackCurrent.evento=sampling; L.push_back(TrackCurrent); 
	L.sort();

}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double g1dshape(double d){

	double k1=50, k2=4;
	return ( k1*(d-dref)+k2*pow((d-dref),3) );
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double g1(double d){

	if(0 && d>dref+0.10*dref){
		printf("\n\ng1(.) d>dref\n\n"); system("pause"); return -1;}

	if(d<27) return -10000; else return g1dshape(d);
	//if(d>40) return 0;
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
//funzione di ordinamento della lista eventi 
bool track::operator < (track T){

	return (istant<T.istant);
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double NumeroRand(double max){

	double out;
	
	double NumeroRand0ToRandMax=rand();
	out=NumeroRand0ToRandMax/RAND_MAX;
	out=out*max;

	return out;
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double Segno(double numero){

	if(numero<0)
		return (-1);
	else
		return (+1);
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
int TrovaIndiceDelMinimo(double *Array, int MaxDim){

double minimo=1000000000000.0;
int indice;
for(int i=0;i<MaxDim;i++)
	if(Array[i]<minimo){
                minimo=Array[i];
				indice=i;
	}
				
return indice;
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double TrovaMinimo(double *Array, int MaxDim){

double minimo=1000000000000.0;
for(int i=0;i<MaxDim;i++)
        if(Array[i]<minimo)
                minimo=Array[i];
return minimo;
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
double TrovaMassimo(double *Array, int MaxDim){

double Max = -1000000000000.0;
for(int i=0;i<MaxDim;i++)
        if(Array[i]>Max)
                Max=Array[i];
return Max;
}
////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////
int TrovaIndiceDelMassimo(double *Array, int MaxDim){

double Max = -1000000000000.0;
int indice;
for(int i=0;i<MaxDim;i++)
	if(Array[i]>Max){
                Max=Array[i];
				indice=i;
	}
				
return indice;
}
////////////////////////////////////////////////////////////////////