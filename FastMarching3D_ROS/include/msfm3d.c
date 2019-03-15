#include "math.h"
#include "common.c"

/*This function MSFM3D calculates the shortest distance from a list of */
/*points to all other pixels in an image, using the */
/*Multistencil Fast Marching Method (MSFM). This method gives more accurate */
/*distances by using second order derivatives and cross neighbours. */
/* */
/*T=msfm3d(F, SourcePoints, UseSecond, UseCross) */
/* */
/*inputs, */
/*   F: The 3D speed image. The speed function must always be larger */
/*			than zero (min value 1e-8), otherwise some regions will */
/*			never be reached because the time will go to infinity.  */
/*  SourcePoints : A list of starting points [3 x N] (distance zero) */
/*  UseSecond : Boolean Set to true if not only first but also second */
/*               order derivatives are used (default) */
/*  UseCross: Boolean Set to true if also cross neighbours */
/*               are used (default) */
/*outputs, */
/*  T : Image with distance from SourcePoints to all pixels */

/* */
/*Function is written by D.Kroon University of Twente (June 2009) */

double second_derivative(double Txm1, double Txm2, double Txp1, double Txp2) {
    bool ch1, ch2;
    double Tm;
    Tm=INF;
    ch1=(Txm2<Txm1)&&IsFinite(Txm1); ch2=(Txp2<Txp1)&&IsFinite(Txp1);
    if(ch1&&ch2) { Tm =min( (4.0*Txm1-Txm2)/3.0 , (4.0*Txp1-Txp2)/3.0);}
    else if(ch1) { Tm =(4.0*Txm1-Txm2)/3.0; }
    else if(ch2) { Tm =(4.0*Txp1-Txp2)/3.0; }
    return Tm;
}

double CalculateDistance(double *T, double Fijk, int *dims, int i, int j, int k, bool usesecond, bool usecross, bool *Frozen) {
    
    /* Loop variables */
    int q, t;
    
    /* Current location */
    int in, jn, kn;
    
    /* Derivatives */
    double Tm[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double Tm2[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double Coeff[3];
    
    /* local derivatives in distance image */
    double Txm1, Txm2, Txp1, Txp2;
    double Tym1, Tym2, Typ1, Typ2;
    double Tzm1, Tzm2, Tzp1, Tzp2;
    /* local cross derivatives in distance image */
    double Tr2t1m1, Tr2t1m2, Tr2t1p1, Tr2t1p2;
    double Tr2t2m1, Tr2t2m2, Tr2t2p1, Tr2t2p2;
    double Tr2t3m1, Tr2t3m2, Tr2t3p1, Tr2t3p2;
    double Tr3t1m1, Tr3t1m2, Tr3t1p1, Tr3t1p2;
    double Tr3t2m1, Tr3t2m2, Tr3t2p1, Tr3t2p2;
    double Tr3t3m1, Tr3t3m2, Tr3t3p1, Tr3t3p2;
    double Tr4t1m1, Tr4t1m2, Tr4t1p1, Tr4t1p2;
    double Tr4t2m1, Tr4t2m2, Tr4t2p1, Tr4t2p2;
    double Tr4t3m1, Tr4t3m2, Tr4t3p1, Tr4t3p2;
    double Tr5t1m1, Tr5t1m2, Tr5t1p1, Tr5t1p2;
    double Tr5t2m1, Tr5t2m2, Tr5t2p1, Tr5t2p2;
    double Tr5t3m1, Tr5t3m2, Tr5t3p1, Tr5t3p2;
    double Tr6t1m1, Tr6t1m2, Tr6t1p1, Tr6t1p2;
    double Tr6t2m1, Tr6t2m2, Tr6t2p1, Tr6t2p2;
    double Tr6t3m1, Tr6t3m2, Tr6t3p1, Tr6t3p2;
    
    double Tt, Tt2;
    
    
    /* Return values root of polynomial */
    double ansroot[2]={0, 0};
    
    /* Order derivatives in a certain direction */
    int Order[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    /* Neighbours 4x2 */
    int ne[18]={-1,  0,  0, 1, 0, 0, 0, -1,  0, 0, 1, 0, 0,  0, -1, 0, 0, 1};
    
    /* Stencil constants */
    double G1[18]={1, 1, 1, 1, 0.5, 0.5, 1, 0.5, 0.5, 1, 0.5, 0.5, 0.5, 0.3333333333333, 0.3333333333333, 0.5, 0.3333333333333, 0.3333333333333};
    double G2[18]={2.250, 2.250, 2.250, 2.250, 1.125, 1.125, 2.250, 1.125, 1.125, 2.250, 1.125, 1.125, 1.125, 0.750, 0.750, 1.125, 0.750, 0.750};
    
    
    /*Get First order derivatives (only use frozen pixel) */
    in=i-1; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txm1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txm1=INF; }
    in=i+1; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txp1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txp1=INF; }
    in=i+0; jn=j-1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tym1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tym1=INF; }
    in=i+0; jn=j+1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Typ1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Typ1=INF; }
    in=i+0; jn=j+0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzm1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzm1=INF; }
    in=i+0; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzp1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzp1=INF; }
    
    if(usecross) {
        Tr2t1m1=Txm1;
        Tr2t1p1=Txp1;
        in=i-0; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2m1=INF; }
        in=i+0; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2p1=INF; }
        in=i-0; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3m1=INF; }
        in=i+0; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3p1=INF; }
        Tr3t1m1=Tym1;
        Tr3t1p1=Typ1;
        in=i-1; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2m1=INF; }
        in=i+1; jn=j+0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2p1=INF; }
        in=i-1; jn=j-0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3m1=INF; }
        in=i+1; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3p1=INF; }
        Tr4t1m1=Tzm1;
        Tr4t1p1=Tzp1;
        in=i-1; jn=j-1; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2m1=INF; }
        in=i+1; jn=j+1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2p1=INF; }
        in=i-1; jn=j+1; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3m1=INF; }
        in=i+1; jn=j-1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3p1=INF; }
        Tr5t1m1=Tr3t3m1;
        Tr5t1p1=Tr3t3p1;
        in=i-1; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2m1=INF; }
        in=i+1; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2p1=INF; }
        in=i-1; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3m1=INF; }
        in=i+1; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3p1=INF; }
        Tr6t1m1=Tr3t2p1;
        Tr6t1p1=Tr3t2m1;
        in=i-1; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2m1=INF; }
        in=i+1; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2p1=INF; }
        in=i-1; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3m1=INF; }
        in=i+1; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3p1=INF; }
    }
    
    /*The values in order is 0 if no neighbours in that direction */
    /*1 if 1e order derivatives is used and 2 if second order */
    /*derivatives are used */
    
    
    /*Make 1e order derivatives in x and y direction */
    Tm[0] = min( Txm1 , Txp1); if(IsFinite(Tm[0])){ Order[0]=1; } else { Order[0]=0; }
    Tm[1] = min( Tym1 , Typ1); if(IsFinite(Tm[1])){ Order[1]=1; } else { Order[1]=0; }
    Tm[2] = min( Tzm1 , Tzp1); if(IsFinite(Tm[2])){ Order[2]=1; } else { Order[2]=0; }
    
    /*Make 1e order derivatives in cross directions */
    if(usecross) {
        Tm[3] = Tm[0]; Order[3]=Order[0];
        Tm[4] = min( Tr2t2m1 , Tr2t2p1); if(IsFinite(Tm[4])){ Order[4]=1; } else { Order[4]=0; }
        Tm[5] = min( Tr2t3m1 , Tr2t3p1); if(IsFinite(Tm[5])){ Order[5]=1; } else { Order[5]=0; }
        
        Tm[6] = Tm[1]; Order[6]=Order[1];
        Tm[7] = min( Tr3t2m1 , Tr3t2p1); if(IsFinite(Tm[7])){ Order[7]=1; } else { Order[7]=0; }
        Tm[8] = min( Tr3t3m1 , Tr3t3p1); if(IsFinite(Tm[8])){ Order[8]=1; } else { Order[8]=0; }
        
        Tm[9] = Tm[2]; Order[9]=Order[2];
        Tm[10] = min( Tr4t2m1 , Tr4t2p1); if(IsFinite(Tm[10])){ Order[10]=1; } else { Order[10]=0; }
        Tm[11] = min( Tr4t3m1 , Tr4t3p1); if(IsFinite(Tm[11])){ Order[11]=1; } else { Order[11]=0; }
        
        Tm[12] = Tm[8]; Order[12]=Order[8];
        Tm[13] = min( Tr5t2m1 , Tr5t2p1); if(IsFinite(Tm[13])){ Order[13]=1; } else { Order[13]=0; }
        Tm[14] = min( Tr5t3m1 , Tr5t3p1); if(IsFinite(Tm[14])){ Order[14]=1; } else { Order[14]=0; }
        
        Tm[15] = Tm[7]; Order[15]=Order[7];
        Tm[16] = min( Tr6t2m1 , Tr6t2p1); if(IsFinite(Tm[16])){ Order[16]=1; } else { Order[16]=0; }
        Tm[17] = min( Tr6t3m1 , Tr6t3p1); if(IsFinite(Tm[17])){ Order[17]=1; } else { Order[17]=0; }
    }
    
    /*Make 2e order derivatives */
    if(usesecond) {
        /*Get Second order derivatives (only use frozen pixel) */
        /*Get First order derivatives (only use frozen pixel) */
        in=i-2; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txm2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txm2=INF; }
        in=i+2; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txp2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txp2=INF; }
        in=i+0; jn=j-2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tym2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tym2=INF; }
        in=i+0; jn=j+2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Typ2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Typ2=INF; }
        in=i+0; jn=j+0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzm2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzm2=INF; }
        in=i+0; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzp2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzp2=INF; }
        
        if(usecross) {
            Tr2t1m2=Txm2;
            Tr2t1p2=Txp2;
            in=i-0; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2m2=INF; }
            in=i+0; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2p2=INF; }
            in=i-0; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3m2=INF; }
            in=i+0; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3p2=INF; }
            Tr3t1m2=Tym2;
            Tr3t1p2=Typ2;
            in=i-2; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2m2=INF; }
            in=i+2; jn=j+0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2p2=INF; }
            in=i-2; jn=j-0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3m2=INF; }
            in=i+2; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3p2=INF; }
            Tr4t1m2=Tzm2;
            Tr4t1p2=Tzp2;
            in=i-2; jn=j-2; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2m2=INF; }
            in=i+2; jn=j+2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2p2=INF; }
            in=i-2; jn=j+2; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3m2=INF; }
            in=i+2; jn=j-2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3p2=INF; }
            Tr5t1m2=Tr3t3m2;
            Tr5t1p2=Tr3t3p2;
            in=i-2; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2m2=INF; }
            in=i+2; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2p2=INF; }
            in=i-2; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3m2=INF; }
            in=i+2; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3p2=INF; }
            Tr6t1m2=Tr3t2p2;
            Tr6t1p2=Tr3t2m2;
            in=i-2; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2m2=INF; }
            in=i+2; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2p2=INF; }
            in=i-2; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3m2=INF; }
            in=i+2; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3p2=INF; }
        }
        
        
        /*pixels with a pixeldistance 2 from the center must be */
        /*lower in value otherwise use other side or first order */
        
        Tm2[0]=second_derivative(Txm1, Txm2, Txp1, Txp2); if(IsInf(Tm2[0])) { Tm2[0]=0; } else { Order[0]=2; }
        Tm2[1]=second_derivative(Tym1, Tym2, Typ1, Typ2); if(IsInf(Tm2[1])) { Tm2[1]=0; } else { Order[1]=2; }
        Tm2[2]=second_derivative(Tzm1, Tzm2, Tzp1, Tzp2); if(IsInf(Tm2[2])) { Tm2[2]=0; } else { Order[2]=2; }
        
        if(usecross) {
            Tm2[3]=Tm2[0]; Order[3]=Order[0];
            Tm2[4]=second_derivative(Tr2t2m1, Tr2t2m2, Tr2t2p1, Tr2t2p2); if(IsInf(Tm2[4])) { Tm2[4]=0; } else { Order[4]=2; }
            Tm2[5]=second_derivative(Tr2t3m1, Tr2t3m2, Tr2t3p1, Tr2t3p2); if(IsInf(Tm2[5])) { Tm2[5]=0; } else { Order[5]=2; }
            
            Tm2[6]=Tm2[1]; Order[6]=Order[1];
            Tm2[7]=second_derivative(Tr3t2m1, Tr3t2m2, Tr3t2p1, Tr3t2p2); if(IsInf(Tm2[7])) { Tm2[7]=0; } else { Order[7]=2; }
            Tm2[8]=second_derivative(Tr3t3m1, Tr3t3m2, Tr3t3p1, Tr3t3p2); if(IsInf(Tm2[8])) { Tm2[8]=0; } else { Order[8]=2; }
            
            Tm2[9]=Tm2[2]; Order[9]=Order[2];
            Tm2[10]=second_derivative(Tr4t2m1, Tr4t2m2, Tr4t2p1, Tr4t2p2); if(IsInf(Tm2[10])) { Tm2[10]=0; } else { Order[10]=2; }
            Tm2[11]=second_derivative(Tr4t3m1, Tr4t3m2, Tr4t3p1, Tr4t3p2); if(IsInf(Tm2[11])) { Tm2[11]=0; } else { Order[11]=2; }
            
            Tm2[12]=Tm2[8]; Order[12]=Order[8];
            Tm2[13]=second_derivative(Tr5t2m1, Tr5t2m2, Tr5t2p1, Tr5t2p2); if(IsInf(Tm2[13])) { Tm2[13]=0; } else { Order[13]=2; }
            Tm2[14]=second_derivative(Tr5t3m1, Tr5t3m2, Tr5t3p1, Tr5t3p2); if(IsInf(Tm2[14])) { Tm2[14]=0; } else { Order[14]=2; }
            
            Tm2[15]=Tm2[7]; Order[15]=Order[7];
            Tm2[16]=second_derivative(Tr6t2m1, Tr6t2m2, Tr6t2p1, Tr6t2p2); if(IsInf(Tm2[16])) { Tm2[16]=0; } else { Order[16]=2; }
            Tm2[17]=second_derivative(Tr6t3m1, Tr6t3m2, Tr6t3p1, Tr6t3p2); if(IsInf(Tm2[17])) { Tm2[17]=0; } else { Order[17]=2; }
        }
        
    }
    
    /*Calculate the distance using x and y direction */
    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fijk),eps));
    
    for (t=0; t<3; t++) {
        switch(Order[t]) {
            case 1:
                Coeff[0]+=G1[t]; Coeff[1]+=-2.0*Tm[t]*G1[t]; Coeff[2]+=pow2(Tm[t])*G1[t];
                break;
            case 2:
                Coeff[0]+=G2[t]; Coeff[1]+=-2.0*Tm2[t]*G2[t]; Coeff[2]+=pow2(Tm2[t])*G2[t];
                break;
        }
    }
    
    
    roots(Coeff, ansroot);
    Tt=max(ansroot[0], ansroot[1]);
    
    /*Calculate the distance using the cross directions */
    if(usecross) {
        
        for(q=1; q<6; q++) {
            /* Original Equation */
            /*    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fijk),eps)) */
            Coeff[0]+=0; Coeff[1]+=0; Coeff[2]+=-1/(max(pow2(Fijk),eps));
            
            for (t=q*3; t<((q+1)*3); t++) {
                switch(Order[t]) {
                    case 1:
                        Coeff[0]+=G1[t]; Coeff[1]+=-2.0*Tm[t]*G1[t]; Coeff[2]+=pow2(Tm[t])*G1[t];
                        break;
                    case 2:
                        Coeff[0]+=G2[t]; Coeff[1]+=-2.0*Tm2[t]*G2[t]; Coeff[2]+=pow2(Tm2[t])*G2[t];
                        break;
                }
            }
            /*Select maximum root solution and minimum distance value of both stensils */
            if(Coeff[0]>0) { roots(Coeff, ansroot); Tt2=max(ansroot[0], ansroot[1]); Tt=min(Tt, Tt2); }
        }
    }
    
    /*Upwind condition check, current distance must be larger */
    /*then direct neighbours used in solution */
    /*(Will this ever happen?) */
    if(usecross) {
        for(q=0; q<18; q++) { if(IsFinite(Tm[q])&&(Tt<Tm[q])) { Tt=Tm[minarray(Tm, 18)]+(1/(max(Fijk,eps)));}}
    }
    else {
        for(q=0; q<3; q++) { if(IsFinite(Tm[q])&&(Tt<Tm[q])) { Tt=Tm[minarray(Tm, 3)]+(1/(max(Fijk,eps)));}}
    }
    
    return Tt;
}