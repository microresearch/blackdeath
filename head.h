#define HOW 100


typedef struct {
  int step;
  int returnval;
  int frequency;
  int amplitude;
  float phase;
  float *table;
  float n;
  int *databuf;
  int ch;int chco; 
}freqtype;

typedef struct {
  int ch;int chco;int step;
  double h,a,b,c,lx0,ly0,lz0;
  int inty;
  int intx;
  int intz;
}rosstype;
	
typedef struct{ double x, y; } Point;

typedef struct {
	double prob[5];
	double coeff[4][6];
	Point p1,p2;
	unsigned char returnvalx;
	unsigned char returnvaly;
	unsigned int ifscount;
	unsigned char step;
	} ifss;

void initifs(ifss* ifs);
void runifs(ifss* ifs);


