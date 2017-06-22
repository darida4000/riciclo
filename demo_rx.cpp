	
  /////////////////////////////////////////////////////////////////////
  
  // RICICLO PROGETTO DIFFERENZIATA
  //
  // 
  
  /////////////////////////////////////////////////////////////////////


	
  /////////////////////////////////////////////////////////////////////
  
  // INCLUDES test quattro
  
  /////////////////////////////////////////////////////////////////////
	


#include <stdlib.h>

#include <stdio.h>
#include <unistd.h>
#include <cmath>
#include <string> 
#include <iostream>
#include <fstream> 

#include "rs232.h"
#include "emd.h"

#include <curl/curl.h>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>





// COSTANTI
#define BUF_SIZE 128
#define IMG_OBJECT 8 // immagini nel database
#define IMG_SCENE 3 // numero fotogrammi
#define ORB_PRECISION 5000 // precisione orb

using namespace cv;
using namespace std;

// PROTOTIPI

bool niceHomography(Mat* H);
double homographyRating(Mat* H);
void readImages();
int findMaxIndex();
void captureImages(int cam);
bool calcolaEmd();


// VARIABILI

int maxImages=IMG_OBJECT; // togliere
int objectIndex=0;
Mat img_object;
Mat img_scene[IMG_SCENE];
VideoCapture cap;
bool found=false;  
Mat img_object_data[IMG_OBJECT];
Mat img_matches[IMG_OBJECT];
double determinanti[IMG_OBJECT][IMG_SCENE];
Ptr<Feature2D> f2d;
Ptr<FeatureDetector> detector;
Ptr<DescriptorExtractor> extractor;
Mat descriptors_object[IMG_OBJECT], descriptors_scene[IMG_SCENE];
std::vector<KeyPoint> keypoints_object[IMG_OBJECT], keypoints_scene[IMG_SCENE];
String nomiRifiuti[IMG_OBJECT];
int colindex=0;
double maxFound=0;




int main()
{
	
  /////////////////////////////////////////////////////////////////////
  
  // DICHIARAZIONI
  
  /////////////////////////////////////////////////////////////////////
	
  // RFID 	
  int i=0;
  int k=0;
  int cport_nr_arduino=16;  
  int bdrate_arduino=57600;  
  int cport_nr=24;  
  int bdrate=19200;
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
  char str_send[BUF_SIZE]; // send data buffer
  unsigned char str_recv[BUF_SIZE]; // recv data buffer
  char codice_tessera_hex[6];
 
 
 
  /////////////////////////////////////////////////////////////////////
  
  // CONTROLLI
  
  /////////////////////////////////////////////////////////////////////
 
 
 
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Problema apertura porta scheda rfid\n");
    return(0);
  }
  
  if(RS232_OpenComport(cport_nr_arduino, bdrate_arduino, mode))
  {
    printf("Problema apertura comunicazione con arduino\n");
    return(0);
  }

  /////////////////////////////////////////////////////////////////////
  
  // INIZIALIZZAZIONI
  
  /////////////////////////////////////////////////////////////////////
 
	// open cv
	
	f2d= ORB::create(); 
    int minHessian = 400;
    detector = ORB::create(ORB_PRECISION);
	BFMatcher matcher(NORM_HAMMING); 
    
	// legge tutte le immagini dal disco
	
    readImages();  

	usleep(1000000);  /* ASPETTA un secondo */

  /////////////////////////////////////////////////////////////////////
  
  // INZIO LOOP PRINIPALE
  
  /////////////////////////////////////////////////////////////////////

while(true)
{
	
  /////////////////////////////////////////////////////////////////////
  
  // ATTESA RFID
  
  /////////////////////////////////////////////////////////////////////


  
  
  while(1)
  {

	int n = RS232_PollComport(cport_nr, str_recv, (int)BUF_SIZE);
	if(n > 0){
      str_recv[n] = 0; 

	  
	  sprintf(codice_tessera_hex,"%02X%02X%02X",str_recv[6],str_recv[7],str_recv[8]);
	  
	  printf("CODICE TESSERA PER DATABASE:%s \n",codice_tessera_hex); 
	  
  
      break; // esco dal ciclo se tessera strisciata
            
	}
	i++;
    i %= 2;
    usleep(1000000);  /* 1 secondo di pausa */

  } // fine ciclo lettura su tessera 
  
  
  /////////////////////////////////////////////////////////////////////
  
  // CHIAMATA CURL
  
  /////////////////////////////////////////////////////////////////////
  
  CURL *curl;
  CURLcode res;
 
  curl = curl_easy_init();
  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, "https://api.thingspeak.com/update?api_key=786D9K3E371MIWY6&field1=15");
  
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
 
    res = curl_easy_perform(curl);
    // controllo errori chiamata
    //if(res != CURLE_OK) cout << "Errore";
   //   fprintf(stderr, "errore: %s\n",
              //curl_easy_strerror(res));
 
    curl_easy_cleanup(curl);
  }
  

  
  /////////////////////////////////////////////////////////////////////
  
  // CICLO COMUNICAZIONE ARDUINO
  
  /////////////////////////////////////////////////////////////////////  
  
  
  strcpy(str_send, (char*)str_recv);  
  i=0;
  while(1)
  {
    RS232_cputs(cport_nr_arduino, str_send); // mando codice ricevuto ad arduino (mi serve ?)
	printf("Invio a Arduino: '%s'\n", str_send);
	usleep(1000000);  /* aspetto un secondo */
	int n = RS232_PollComport(cport_nr_arduino, str_recv, (int)BUF_SIZE);
	if(n > 0){
      str_recv[n] = 0;   /* zero alla fine della stringa */
      
            
      printf("ricevuti %i bytes: '%s'\n", n, (char *)str_recv);
      
            
	}
	i++;
    
    usleep(5000000);  /* pausa 5 secondi */
    if(i==2) break; // esce
  }
  
} // fine loop principale





  return(0);
}



  /////////////////////////////////////////////////////////////////////
  
  // FUNZIONI APPOGGIO
  
  /////////////////////////////////////////////////////////////////////  

void readImages()
{

  img_object_data[0] = imread("img/mk.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[0] = "Milka";
  img_object_data[1] = imread("img/cr2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[1] = "Croccantelle";
  img_object_data[2] = imread("img/cr1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[2] = "Croccantelle";
  img_object_data[3] = imread("img/sc2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[3] = "Schiacciatelle";
  img_object_data[4] = imread("img/fz1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[4] = "Fonzie";
  img_object_data[5] = imread("img/ps1.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[5] = "Pizzottelle";
  img_object_data[6] = imread("img/sc3.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[6] = "Schiacciatelle";
  img_object_data[7] = imread("img/fz2.jpg" , CV_LOAD_IMAGE_GRAYSCALE );
  nomiRifiuti[7] = "Fonzie";

  for(int i =0;i< maxImages; i++) for(int ii =0;ii< 3; ii++) determinanti[i][ii] = 0;
 
	for (int i=0;i< maxImages;i++)
	{	
		/*detector->detect(img_object_data[i], keypoints_object[i]);
		extractor->compute(img_object_data[i], keypoints_object[i], descriptors_object[i] );
		*/
		detector->detectAndCompute(img_object_data[i], noArray(), keypoints_object[i], descriptors_object[i]);	
		/*if(descriptors_object[i].type()!=CV_32F) {
          descriptors_object[i].convertTo(descriptors_object[i], CV_32F);
        }*/

        
	}            
}

void captureImages(int cam)
{

	cout << "Inizio cattura img";
	try
	{
		if(!cap.open(cam))
		{
			cout << "Errore apertura cam " << cam;
        }
    }
    catch(cv::Exception& e)
    {
		sleep(3000);
		if(!cap.open(cam))
		{
			cout << "Errore apertura cam " << cam;
			return;
        }		
	}
	

    try
    {
      for(int i=0;i<10;i++) cap >> img_scene[0];
      for(int i=0;i<10;i++) cap >> img_scene[1];
      for(int i=0;i<10;i++) cap >> img_scene[2];
    } 
          catch(cv::Exception& e)
    {


    } 
    /*
    // controllo in range
    
    Mat test2_mask,hsv_test2;
    cvtColor( img_scene[2], hsv_test2, COLOR_BGR2HSV );

    inRange(hsv_test2, Scalar(25, 15, 50), Scalar(33, 255, 255), test2_mask); // valore per fonzie con percentuali intorno al 15
    //inRange(hsv_test2, Scalar(10, 100, 100), Scalar(20, 255, 255), test2_mask); // valore per croccantelle con percentuali intorno al 15
    
    double pixel = countNonZero(test2_mask); //  307200 pixel totali per immagini 640 x 480
    cout << "non zero:" << pixel << endl;
    
    double perc = pixel*100/307200;
    if (perc > 15) return 5;
    
    */
	cout << "immagini catturate";
	for (int i=0;i< 3;i++)
	{	
	
		detector->detectAndCompute(img_scene[i], noArray(), keypoints_scene[i], descriptors_scene[i]);	

        /*if(descriptors_scene[i].type()!=CV_32F) {
          descriptors_scene[i].convertTo(descriptors_scene[i], CV_32F);
        }*/
	
	}            	
	imshow("test",img_scene[0]);
	

    waitKey(0);

}


bool niceHomography(Mat* H)
{
	const double det = H->at<double>(0,0)* H->at<double>(1,1)-H->at<double>(1,0)* H->at<double>(0,1);
	if (det<0) return false;
	const double N1 = sqrt(H->at<double>(0,0)* H->at<double>(0,0)+H->at<double>(1,0)* H->at<double>(0,1));
	if (N1>4 || N1 < 0.1) return false;
	const double N2 = sqrt(H->at<double>(0,1)* H->at<double>(0,1)+H->at<double>(1,1)* H->at<double>(1,1));
	if (N2>4 || N2 < 0.1) return false;
	const double N3 = sqrt(H->at<double>(2,0)* H->at<double>(2,0)+H->at<double>(2,1)* H->at<double>(2,1));
	if (N3>0.002) return false;

	return true;


}

double homographyRating(Mat* H)
{

    const double det = H->at<double>(0,0)* H->at<double>(1,1)-H->at<double>(1,0)* H->at<double>(0,1);
    return det;

}
int findMaxIndex()
{

    /*cout << "determinanti:" << endl;
    cout << "--------------" << endl;
*/
    double max=0;
    int indice = -1;
    int k;
    for( k=0;k< maxImages;k++)
    {
      
      for(int h=0;h<3;h++)
      {
  //      cout << " " << determinanti[k][h];
        if (determinanti[k][h] > 1) continue;
        if(determinanti[k][h] > max)
        {

          max = determinanti[k][h];
          indice = k;
          colindex=h; 
        }
      }
      cout << endl;
    }
    maxFound=max;
    return indice;

}
bool calcolaEmd()
{

    Mat bg;
    Mat src_base, hsv_base;
    Mat src_test1, hsv_test1;
    Mat src_test2, hsv_test2;
    Mat hsv_half_down;

    
    int imgCarta=2;
    Mat carta[imgCarta];
    Mat scene[3];
    Mat hsv_carta[imgCarta];
    Mat hsv_scene[3];
    Mat mask_carta[imgCarta];
    Mat mask_scene[3];
 
    // carico immagine sfondo
    bg=imread("sfondo.jpg");
    
    carta[0] = imread( "cartamarroneok.jpg", 1 );
    carta[1] = imread( "cartamarroneok.jpg", 1 ); // sostituire con carta bianca
    
    for(int i=0;i<imgCarta;i++) 
      {
        carta[i] = carta[i] - bg;
      
      /// Converto in HSV
        cvtColor( carta[i], hsv_carta[i], COLOR_BGR2HSV );
        
 
    
    inRange(hsv_carta[i], Scalar(0, 15, 50), Scalar(180, 255, 255), mask_carta[i]);
 
      }

    for(int i=0;i<3;i++) 
      {
        scene[i] = img_scene[i].clone();
        scene[i] = scene[i] - bg;
        cvtColor( scene[i], hsv_scene[i], COLOR_BGR2HSV );
        inRange(hsv_scene[i], Scalar(0, 15, 50), Scalar(180, 255, 255), mask_scene[i]);
      }


   // setup
    int h_bins = 20; int s_bins = 20; int v_bins =50;
    int histSize[] = { h_bins, s_bins };

    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    float v_ranges[] = { 100, 256 };

    const float* ranges[] = { h_ranges,  s_ranges };

    
    int channels[] = { 0,1 };

    MatND hist_base[imgCarta];
    MatND hist_test[3];

    for (int i=0;i<imgCarta;i++)
    {
      calcHist( &hsv_carta[i], 1, channels, mask_carta[i], hist_base[i], 2, histSize, ranges, true, false );
      normalize( hist_base[i], hist_base[i], 0, 1, NORM_MINMAX, -1, Mat() );
    }
    for (int i=0;i<3;i++)
    {
      calcHist( &hsv_scene[i], 1, channels, mask_scene[i], hist_test[i], 2, histSize, ranges, true, false );
      normalize( hist_test[i], hist_test[i], 0, 1, NORM_MINMAX, -1, Mat() );
    }


  // calcolo EMD
  for(int i=0;i<imgCarta;i++)   /// per ogni img carta
  {
  for (int j=0;j<3;j++) // per ogni fotogramma
  {  
  vector<cv::Mat> sig(3);
  MatND hist[2];

  hist[0] = hist_base[i].clone();
  hist[1] = hist_test[j].clone();

  for(int i = 0;i<2;i++)
  {
    vector <cv::Vec3f> sigv;
    normalize( hist[i], hist[i], 1, 0, NORM_L1 );
  
    for(int h=0;h<h_bins;h++)
    {
      for(int s=0;s< s_bins;s++)
       {

        float bin_val=hist[i].at<float>(h,s);
        if(bin_val!=0) sigv.push_back(cv::Vec3f(bin_val,(float)h,(float)s));

      }
      sig[i] =cv::Mat(sigv).clone().reshape(1);
       
        
    }
    if(i>0){

        float emdResult = EMD(sig[0],sig[i],cv::DIST_L1);
        cout << "Risultato emd: " << emdResult;
        if (emdResult<4) return true;   
      }
    } 
   }
 } 


return false;
}

