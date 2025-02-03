// C++ code
//Defining PI to be used with PlotCircle1
#define PI 3.14159265


//Section 3 Part 1
//MySum1 Function, receives array and array size
//Returns the sum of all values in the array
int mysum1(int array_in[],int array_size){
  int sum = 0;
  for (int i = 0; i < array_size; i++){
  	sum += array_in[i];
  }
	return sum;
}


//Section 3 Part 2
//MyFib1, receives an interger
//Returns the nth Fibonacci number
int myfib1(int num_in){
  if (num_in > 1){		//when n > 1 recursion needed
  	return myfib1(num_in - 1) + myfib1(num_in -2); //Fib formula loops until n reaches first two elements
  } else if (num_in == 0){  // First element of fib sequence (0)
  	return 0;
  } else if (num_in == 1){ //  Second element of fib sequence (1)
  	return 1;
  }
}


//Section 4 Part 1 (plot circle)
void plotcircle1(int radius_in){
  //Initializing values to hold 4 coordinates to be generated  
  float x_coordinates[4];
  float y_coordinates[4];
  
  //List of Thetas to be used for calculations
  float thetaValues[] = {0.0, PI/2, PI, (3 * PI)/2};
  int theta_values_length = sizeof(thetaValues) / sizeof(thetaValues[0]);
 
  //Calculate an x and y coordinate with their respective formula
  //Make a calculation for each Theta in the thetaValues array
  for (int i = 0; i < theta_values_length ; i++){
  	float x_coordinate_calculation = radius_in * cos(thetaValues[i]);
    float y_coordinate_calculation = radius_in * sin(thetaValues[i]);

    //For each iteration a new set of x and y coordinates are added
    x_coordinates[i] = x_coordinate_calculation;                                      
    y_coordinates[i] = y_coordinate_calculation;
    
    //Searching for -0 values and making them 0 for output reasons
    if(fabs(x_coordinates[i]) < 1e-6) x_coordinates[i] = 0.0;
    if(fabs(y_coordinates[i]) < 1e-6) y_coordinates[i] = 0.0;                                       
  }
  
  
  Serial.println("PlotCircle1 Implementation Tests: ");
  //Printing x coordinates with desired format
  Serial.println("Printing X coordinates: ");
  Serial.print("[ ");
  for (int i = 0; i < theta_values_length ; i++){
    if(i == theta_values_length -1){
   		Serial.print(x_coordinates[i], 1);
      	Serial.print("]");
    }else{
        Serial.print(x_coordinates[i], 1);
    	Serial.print(", ");
    }
  }
  Serial.println();

  //Printing y coordinates with desired format
  Serial.println("Printing Y coordinates: ");
  Serial.print("[ ");
  for (int i = 0; i < theta_values_length ; i++){
    if(i == theta_values_length -1){
   		Serial.print(y_coordinates[i], 1);
    }else{
        Serial.print(y_coordinates[i], 1);
    	Serial.print(", ");
    }
  }
  Serial.println("]");  
  Serial.println();
  	
  //Used to develop functionality systematically
  //int y = 0;
  //Serial.println("y1: ");
  //Serial.println(y = radius_in * sin(0));
  
  //Serial.println("y2: ");
  //Serial.println(y = radius_in * sin(PI/2));
      
  //Serial.println("y3: ");
  //Serial.println(y = radius_in * sin(PI));
  
  //Serial.println("y4: ");
  //Serial.println(y = radius_in * sin((3 * PI)/2 ));
  
  //int x = 0;
  //Serial.println("x1: ");
  //Serial.println(x = radius_in * cos(0));
  
  //Serial.println("x2: ");
  //Serial.println(x = radius_in * cos(PI/2));
  
  //Serial.println("x3: ");
  //Serial.println(x = radius_in * cos(PI));
  
  //Serial.println("x4: ");
  //Serial.println(x = radius_in * cos((3 * PI)/2 ));
}


//Section 4 Part 2
//generate 50 random numbers, each between 1 and 100
void plotrand1(){
  randomSeed(analogRead(0));	//makes randomness more random
  
  Serial.println("PlotRand1 Function Execution: ");
  for (int i = 1; i < 51; i++){
    Serial.print("Rand Val ");
    Serial.print(i);
    Serial.print(": ");
    
    //random(min,max+1)
  	Serial.println(random(1, 101));
  }
}


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
  
  //Section 3 Part 1
  //mysum1 implementation
  //Sum Test Case 1
  int test_array_1[] = {8};
  int test_1_length = sizeof(test_array_1) / sizeof(test_array_1[0]);
  
  //case 2
  int test_array_2[] = {-3,-7,-1};
  int test_2_length = sizeof(test_array_2) / sizeof(test_array_2[0]);

  //case 3
  int test_array_3[] = {0,0,0};
  int test_3_length = sizeof(test_array_3) / sizeof(test_array_3[0]);

  //case 4
  int test_array_4[] = {};
  int test_4_length = sizeof(test_array_4) / sizeof(test_array_4[0]);

  //case 5
  int test_array_5[] = {-4};
  int test_5_length = sizeof(test_array_5) / sizeof(test_array_5[0]);
  
  //case 6
  int test_array_6[] = {-1, 1, 2, -2, 3};
  int test_6_length = sizeof(test_array_6) / sizeof(test_array_6[0]);

  //printing results from mysum1 calls
  Serial.println("MySum1 Implementation Tests: ");
  Serial.print("Sum Test 1 Results: ");
  Serial.println(mysum1(test_array_1, test_1_length));
  Serial.print("Sum Test 2 Results: ");
  Serial.println(mysum1(test_array_2, test_2_length));
  Serial.print("Sum Test 3 Results: ");
  Serial.println(mysum1(test_array_3, test_3_length));
  Serial.print("Sum Test 4 Results: ");
  Serial.println(mysum1(test_array_4, test_4_length));
  Serial.print("Sum Test 5 Results: ");
  Serial.println(mysum1(test_array_5, test_5_length));
  Serial.print("Sum Test 6 Results: ");
  Serial.println(mysum1(test_array_6, test_6_length));
  Serial.println();
  
  
  //Section 3 Part 2
  //myfib1 implementation 
  Serial.println("MyFib1 Implementation Tests: ");
  //Fib Test case 1
  int test_num_1 = 2;
  Serial.print("Fib Test 1 Results: ");
  Serial.println(myfib1(test_num_1));
  
  //case 2
  int test_num_2 = 8;
  Serial.print("Fib Test 2 Results: ");
  Serial.println(myfib1(test_num_2));
  
  //case 3
  int test_num_3 = 11;
  Serial.print("Fib Test 3 Results: ");
  Serial.println(myfib1(test_num_3));
  
  //case 4
  int test_num_4 = 15;
  Serial.print("Fib Test 4 Results: ");
  Serial.println(myfib1(test_num_4));
  Serial.println();
  
  
  //Section 4 Part 1
  //Plot Circle Pseudocode
  //compute 2 arrays of data points
  //one for x coordinate
  //one for y coordinate
  //given radius (r)
  //Theta, the angle of radians (0 < theta < 2pi)
  //Need to do this for 4 values of the full circle
  // x = radius * cos(theta);
  // y = radius * sin(theta);
  //with r = 1 , x = [1, 0, -1, 0]
  //with r = 1 , y = [0, 1, 0, -1]
  
  int radius = 1;
  plotcircle1(radius);
  plotcircle1(5);
  
  //Section 4 Part 2
  //Generate 50 rand numbers between 1 and 100
  plotrand1();
}
