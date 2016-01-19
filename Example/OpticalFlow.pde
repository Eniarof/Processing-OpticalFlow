// Optical Flow 2010/05/28
// Hidetoshi Shimodaira shimo@is.titech.ac.jp 2010 GPL
// http://www.openprocessing.org/sketch/10435
// Modifications 2016/01/19 by ENIAROF

///////////////////////////////////////////////
int wscreen=640;
int hscreen=480;
int gs=10; // grid step (pixels)
//float predsec=1.0; // prediction time (sec): larger for longer vector
float predsec=0.25; // prediction time (sec): larger for longer vector
///////////////////////////////////////////////

color[] vline;

// grid parameters
int as=gs*2;  // window size for averaging (-as,...,+as)
int gw=wscreen/gs;
int gh=hscreen/gs;
int gc = gw * gh;
int gs2=gs/2;
float df=predsec*30; // 30 fps

// regression vectors
float[] fx, fy, ft;
int fm=3*9; // length of the vectors

// regularization term for regression
float fc=pow(10, 8); // larger values for noisy video

// smoothing parameters
float wflow=0.1; // smaller value for longer smoothing

// switch
boolean flagmirror=true; // mirroring image?

// internally used variables
float ar, ag, ab; // used as return value of pixave
float[] dtr, dtg, dtb; // differentiation by t (red,gree,blue)
float[] dxr, dxg, dxb; // differentiation by x (red,gree,blue)
float[] dyr, dyg, dyb; // differentiation by y (red,gree,blue)
float[] par, pag, pab; // averaged grid values (red,gree,blue)
float[] flowx, flowy; // computed optical flow
float[] sflowx, sflowy; // slowly changing version of the flow

float meanX;
float meanY;
float angle;
float magnitude;

void setupFlow(int w, int h) {

  video = new Capture(this, w, h);
  video.start();

  // arrays
  par = new float[gc];
  pag = new float[gc];
  pab = new float[gc];
  dtr = new float[gc];
  dtg = new float[gc];
  dtb = new float[gc];
  dxr = new float[gc];
  dxg = new float[gc];
  dxb = new float[gc];
  dyr = new float[gc];
  dyg = new float[gc];
  dyb = new float[gc];
  flowx = new float[gc];
  flowy = new float[gc];
  sflowx = new float[gc];
  sflowy = new float[gc];
  fx = new float[fm];
  fy = new float[fm];
  ft = new float[fm];
  vline = new color[wscreen];
}


void updateFlow() {

  if (!video.available()) return;

  // video capture
  video.read();

  // mirror
  if (flagmirror) {
    for (int y=0; y<hscreen; y++) {
      int ig=y*wscreen;
      for (int x=0; x<wscreen; x++)
        vline[x] = video.pixels[ig+x];
      for (int x=0; x<wscreen; x++)
        video.pixels[ig+x]=vline[wscreen-1-x];
    }
  }

  // 1st sweep : differentiation by time
  for (int ix=0; ix<gw; ix++) {
    int x0=ix*gs+gs2;
    for (int iy=0; iy<gh; iy++) {
      int y0=iy*gs+gs2;
      int ig=iy*gw+ix;
      // compute average pixel at (x0,y0)
      pixave(x0-as, y0-as, x0+as, y0+as);
      // compute time difference
      dtr[ig] = ar-par[ig]; // red
      dtg[ig] = ag-pag[ig]; // green
      dtb[ig] = ab-pab[ig]; // blue
      // save the pixel
      par[ig]=ar;
      pag[ig]=ag;
      pab[ig]=ab;
    }
  }

  // 2nd sweep : differentiations by x and y
  for (int ix=1; ix<gw-1; ix++) {
    for (int iy=1; iy<gh-1; iy++) {
      int ig=iy*gw+ix;
      // compute x difference
      dxr[ig] = par[ig+1]-par[ig-1]; // red
      dxg[ig] = pag[ig+1]-pag[ig-1]; // green
      dxb[ig] = pab[ig+1]-pab[ig-1]; // blue
      // compute y difference
      dyr[ig] = par[ig+gw]-par[ig-gw]; // red
      dyg[ig] = pag[ig+gw]-pag[ig-gw]; // green
      dyb[ig] = pab[ig+gw]-pab[ig-gw]; // blue
    }
  }

  // 3rd sweep : solving optical flow
  for (int ix=1; ix<gw-1; ix++) {
    int x0=ix*gs+gs2;
    for (int iy=1; iy<gh-1; iy++) {
      int y0=iy*gs+gs2;
      int ig=iy*gw+ix;

      // prepare vectors fx, fy, ft
      getnext9(dxr, fx, ig, 0); // dx red
      getnext9(dxg, fx, ig, 9); // dx green
      getnext9(dxb, fx, ig, 18);// dx blue
      getnext9(dyr, fy, ig, 0); // dy red
      getnext9(dyg, fy, ig, 9); // dy green
      getnext9(dyb, fy, ig, 18);// dy blue
      getnext9(dtr, ft, ig, 0); // dt red
      getnext9(dtg, ft, ig, 9); // dt green
      getnext9(dtb, ft, ig, 18);// dt blue

      // solve for (flowx, flowy) such that
      // fx flowx + fy flowy + ft = 0
      solveflow(ig);

      // smoothing
      sflowx[ig]+=(flowx[ig]-sflowx[ig])*wflow;
      sflowy[ig]+=(flowy[ig]-sflowy[ig])*wflow;
    }
  }
  
  // calculate mean
  
  meanX = 0;
  meanY = 0;
  
  for(int i=0; i<gc; i++) {
    meanX += flowx[i];
    meanY += flowy[i];
  }
  
  meanX /= gc;
  meanY /= gc;
  
  angle = atan2(meanY,meanX);
  magnitude = mag(meanX,meanY);
  
}


void drawMean() {
  
    if (magnitude < 0.25) return;
    
    strokeWeight(5);
    stroke(255,255,0);
    
    pushMatrix();
    translate(width/2,height/2);
    rotate(angle);
    float radius = magnitude*100.0;
    line(0,0,radius,0);
    line(radius,0,radius-15,-15);
    line(radius,0,radius-15,15);
    popMatrix();
  
}


void drawFlow() {

  set(wscreen, 0, video);

  // 5th sweep : draw the flow
  for (int ix=0; ix<gw; ix++) {
    int x0=ix*gs+gs2;
    for (int iy=0; iy<gh; iy++) {
      int y0=iy*gs+gs2;
      int ig=iy*gw+ix;

      float u=df*sflowx[ig];
      float v=df*sflowy[ig];

      // draw the line segments for optical flow
      float a=sqrt(u*u+v*v);
      if (a>=2.0) { // draw only if the length >=2.0
        stroke(127, 255*a);
        strokeWeight(1);
        line(x0, y0, x0+u, y0+v);
      }
    }
  }
  
}


// calculate average pixel value (r,g,b) for rectangle region
void pixave(int x1, int y1, int x2, int y2) {
  float sumr, sumg, sumb;
  color pix;
  int r, g, b;
  int n;

  if (x1<0) x1=0;
  if (x2>=wscreen) x2=wscreen-1;
  if (y1<0) y1=0;
  if (y2>=hscreen) y2=hscreen-1;

  sumr=sumg=sumb=0.0;
  for (int y=y1; y<=y2; y++) {
    for (int i=wscreen*y+x1; i<=wscreen*y+x2; i++) {
      pix=video.pixels[i];
      b=pix & 0xFF; // blue
      pix = pix >> 8;
      g=pix & 0xFF; // green
      pix = pix >> 8;
      r=pix & 0xFF; // red
      // averaging the values
      sumr += r;
      sumg += g;
      sumb += b;
    }
  }
  n = (x2-x1+1)*(y2-y1+1); // number of pixels
  // the results are stored in static variables
  ar = sumr/n;
  ag=sumg/n;
  ab=sumb/n;
}

// extract values from 9 neighbour grids
void getnext9(float x[], float y[], int i, int j) {
  y[j+0] = x[i+0];
  y[j+1] = x[i-1];
  y[j+2] = x[i+1];
  y[j+3] = x[i-gw];
  y[j+4] = x[i+gw];
  y[j+5] = x[i-gw-1];
  y[j+6] = x[i-gw+1];
  y[j+7] = x[i+gw-1];
  y[j+8] = x[i+gw+1];
}

// solve optical flow by least squares (regression analysis)
void solveflow(int ig) {
  float xx, xy, yy, xt, yt;
  float a, u, v, w;

  // prepare covariances
  xx=xy=yy=xt=yt=0.0;
  for (int i=0; i<fm; i++) {
    xx += fx[i]*fx[i];
    xy += fx[i]*fy[i];
    yy += fy[i]*fy[i];
    xt += fx[i]*ft[i];
    yt += fy[i]*ft[i];
  }

  // least squares computation
  a = xx*yy - xy*xy + fc; // fc is for stable computation
  u = yy*xt - xy*yt; // x direction
  v = xx*yt - xy*xt; // y direction

  // write back
  flowx[ig] = -2*gs*u/a; // optical flow x (pixel per frame)
  flowy[ig] = -2*gs*v/a; // optical flow y (pixel per frame)
}