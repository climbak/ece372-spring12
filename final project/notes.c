#define b                               150 //one beat length in ms
#define b2                              2*b
#define b3                              3*b
#define b4                              4*b
#define b43                             4*b/3
#define b23                             2*b/3
#define hb                              b/2
#define u                               10 //10 ms delay for between same notes


//volatile int numNotes = 60;
//volatile int noteTime = 500;

//volatile int noteTime[numNotes] = {b,b,b,b,b,b,b,b,b,b};

//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b3,b,b3,b,b2,b,b2,b,b2,b,b,b,b,b,b,b,b43,b43,b43,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,b2,b,b,b,b,b,b,b,b43,b43,b43,b,b,b,b,b,b,b,b,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b3,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,b*7,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,*********b,b3,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b2,b,b2,b,7*b,b,u,b,b,b,b,b,b,b,b,b,b,b,b,b3,b,u,b,b,b,b,b,b,b,8*b,b,u,b,b,b,b,b,b,b,b,b,b,b,b};
volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,
                                                                   b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b2,b4,
                                                                   b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,b,b,b,b2,b,b,b2,b4,
                                                                   b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,u,b,b,b,b,b,b,b,b4,b4,b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2};
//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,****b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b,****b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****
//                                      ****page 2**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//                                      ****page 3**** b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****
//                                      ****page 4**** b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2,****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2}
//volatile int noteTime[numNotes] = {b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2,****b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b,b,b,b,b2,b,b,b,b,b,b,b,b,b,b****page 2****,b2,b2,b2,b,b,b,b,b,b,b,b,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2****,b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 3****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b,b,b,b,b,b,u,b,b,b2,****b2,b,b,b,b,b,b,b,b,b,b,b,b,b,b,****b2,b,b,b,b,b2,b,b,b2,b4,****page 4****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****bu,,b,b,b,b,b,b,b,b4,b4,****b,u,b,b,b,b,b,b,b,b,b,b,b,b,b,b2****b,u,b,b,b,b,b,b,b,b,b,b2,b,b,b2};
/*
volatile int noteTime[numNotes] = {240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,
                                                        240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500};
*/                                              


/************************************************************************************************************************************
 ************************************************************  FLAGPOLE  ************************************************************
 ************************************************************************************************************************************/

volatile int flagpoleNumNotes = 29;
volatile int flagpoleNoteTimes[flagpoleNumNotes] = {b23,b23,b23,b23,b23,b23,b2,b2,b23,b23,b23,b23,b23,b23,b2,b2,b23,b23,b23,b23,b23,b23,b2,b23,u,b23,u,b23,b4};
float flagpoleNotes[flagpoleNumNotes] = {G/2,C,E,G,C*2,E*2,G*2,E*2,Ab,C,Eb,Ab*2,C*2,Eb*2,Ab*4,Eb*2,Bb,D,F,Bb*2,D*2,F*2,Bb*4,B*4,u,B*4,u,B*4,C*4};


/************************************************************************************************************************************
 ***********************************************************  UNDERWORLD  ***********************************************************
 ************************************************************************************************************************************/

volatile int underworldNumNotes = 62;
volatile int underworlNoteTimes[underworldNumNotes] = {b,b,b,b,b,b,6*b,
														b,b,b,b,b,b,6*b,
														b,b,b,b,b,b,6*b,
														b,b,b,b,b,b,4*b,b23,b23,b23,
														b2,u,b2,u,b2,u,b2,u,b2,u,b2,u,
														b23,b23,b23,b23,b23,b23,
														b43,u,b43,u,b43,u,b43,u,b43,u,b43,u,
														12*b};
float underworldNotes[underworldNumNotes] = {C,C*2,A,C*2,Bb,Bb*2,R,
											C,C*2,A,C*2,Bb,Bb*2,R,
											F/2,F,D/2,D,Eb/2,Eb,R,
											F/2,F,D/2,D,Eb/2,Eb,R,Eb,D,Db,
											C,u,Eb,u,D,u,Ab,u,G/2,u,Db,u,
											C,Gb,F,E,Bb*2,A*2,
											Ab*2,u,Eb,u,B,u,Bb,u,A,u,Ab,u,
											R};


/************************************************************************************************************************************
 *************************************************************  STARMAN  ************************************************************
 ************************************************************************************************************************************/

volatile int starmanNumNotes = 31;
volatile int starmanNoteTimes[starmanNumNotes] = {b,u,b,u,b,u,hb,hb,hb,b,u,hb,hb,hb,b,b,u,b,u,b,u,hb,hb,hb,b,u,hb,hb,hb,b,u};
float starmandNotes[starmanNumNotes] = {C*2,u,C*2,u,C*2,u,D,C*2,R,C*2,u,D,D*2,D,C,B*2,u,B*2,u,B*2,u,C,B*2,R,B*2,u,C,B*2,C,B*2,u};










float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
                                   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
                                   R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
                                   C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
//      float notes[numNotes] =   {E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R,****C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A*2,R,****G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R****,C*2,R,R,G,R,E,R,R,A*2,R,B*2,R,Bb*2,A,R****,G,E*2,G*2,A*4,R,F*2,G*2,R,E*2,R,C*2,D*2,B*2,R,
//                              ****page 2**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//                              ****page 3**** R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,C*4,R,C*4,u,C*4,R,R****,R,G*2,Gb*2,F*2,Ds*2,R,E*2,R,Gs,A*2,C*2,R,A*2,C*2,D*2,****R,Eb*2,R,R,D*2,R,C*2,R,R,R,
//                              ****page 4**** C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,C*2,u,C*2,R,C*2,R,C*2,D*2,E*2,R,R,****C*2,u,C*2,R,C*2,R,C*2,D*2,R,E*2,C*2,R,A*2,G,R,R****,E*2,u,E*2,R,E*2,R,C*2,E*2,R,G*2,R,R,G,R,R};
       

        /*
        volatile int noteTime[numNotes] = {240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,
                                                        240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500,240,10,250,240,10,250,240,10,250,500};
        

        float notes[numNotes] = {C   ,50,C  ,G  ,50,G  ,A  ,50,A  ,G  ,F  ,50,F  ,E  ,50,E  ,D  ,50,D  ,C  ,G  ,50,G  ,F  ,50,F  ,E  ,50,E  ,D  ,
                                                        G  ,50,G  ,F  ,50,F  ,E  ,50,E  ,D  ,C  ,50,C  ,G  ,50,G  ,A  ,50,A  ,G  ,F  ,50,F  ,E  ,50,E  ,D  ,50,D  ,C  };
        */