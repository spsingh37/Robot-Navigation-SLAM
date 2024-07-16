/**
 * This file is for testing the "matrix.h" ("matrix.c") math code
 * VERSION 5/18/2022
 * AUTHORS teddixon@umich.edu
 * 
 * NOTICE!
 * -currently most null pointer tests fail, this issue has been adressed in meetings
 * -a single matrix greater than 127x127 causes the pico to run out of memory
 * 
 * 
 */

//required libraries
#include <stdio.h>
#include <pico/stdlib.h>
#include <rc/math/matrix.h>
#include <rc/math/vector.h>
#include <string.h>
#include <math.h>

//define ANSI colors for printing
#define RED "\x1b[31m"
#define GRN "\x1b[32m"
#define YLW "\x1b[33m"
#define BLU "\x1b[34m"
#define CYN "\x1b[36m"
#define RST "\x1b[0m"

//define varaiable to hide cmake warning label "[-Wunused-label]"
#define hide_unused_error __attribute__((unused))

//define tolerance for use in non-exact check value tests
#define tolerance 1E-8





//pause until user input function
void pause() { //pause until user hits enter
    printf("\n\nPRESS ENTER");
    getc(stdin);
}

//generic test function
void test(char str[], bool cond) { //test evaluation and printer
    if (cond) {
        printf(GRN "\n\t\t-%s" RST, str);
        sleep_ms(150);
    } else {
        printf(RED "\n[TEST FAILED] TEST: %s" RST, str);
        sleep_ms(300);
    }
}

//identity test function: tests if a matrix is the identity matrix
void test_identity(char str[], rc_matrix_t c) { //matrix test evaluation at every index and printer
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; i<(i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if( (i==j && c.d[i][j]==1) || (i!=j && c.d[i][j]==0) ) {
                //NOOP
            } else {
                flag = 1; //if a diagonal index is not 1 or a non diagonal is not 0, trip the flag
            }
        }
    }
    
    char strConcatTo[25] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "identity test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//random test function: tests if a matrix is within the bounds of a random matrix
void test_random(char str[], rc_matrix_t c) {
    int flag_a = 0; //test failed flag
    int flag_b = 0; //test likely failed flag

    //double nested matrix index checker, break condition when both flags tripped or no more indexes
    for (int i=0; (i<c.rows&&(flag_a==0||flag_b==0)); i++) {
        for (int j=0; (j<c.cols&&(flag_a==0||flag_b==0)); j++) {
            if ( c.d[i][j] > -1 && c.d[i][j] < 1 ) {
                //NOOP
            } else {
                flag_a = 1; //if any index is greater than 1 or less than -1, trip the flag_a
            }
            if ( c.d[i][j] != 0.0 ) {
                //NOOP
            } else {
                flag_b = 1; //if an index is zero, trip the flab_b
            }
        }
    }
    
    char strConcatTo_a[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo_a, "random range test ");
    strcat(strConcatTo_a, str);
    test(strConcatTo_a, flag_a==0); //test that the flag_a was not tripped

    char strConcatTo_b[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo_b, "no zeros test ");
    strcat(strConcatTo_b, str);
    test(strConcatTo_b, flag_b==0); //test that the flag_b was not tripped, note that a false positive is very unlikely but possible for this test
}

//diagonal test function: tests if a diagonal is non-zero and the non-diagonal is zero in a matrix
void test_diagonal(char str[], rc_matrix_t c) { //matrix test evaluation at every index and printer
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; i<(i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if( (i==j && c.d[i][j]!=0) || (i!=j && c.d[i][j]==0) ) {
                //NOOP
            } else {
                flag = 1; //if a diagonal index is not random or a non diagonal is random, trip the flag
            }
        }
    }
    
    char strConcatTo[25] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "diagonal test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped, note that a false positive is very unlikely but possible for this test
}

//test compare function: test that two matricies are identical
void test_compare(char str[], rc_matrix_t c, rc_matrix_t d) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; i<(i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if( (c.d[i][j]== d.d[i][j]) ) {
                //NOOP
            } else {
                flag = 1; //if an index is not identical, trip the flag
            }
        }
    }
    
    char strConcatTo[25] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "identical test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped, note that a false positive is very unlikely but possible for this test
}

//test zeros function: test that a matrix is all zeros
void test_zeros(char str[], rc_matrix_t c) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; i<(i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if( (c.d[i][j]== 0) ) {
                //NOOP
            } else {
                flag = 1; //if an index is not 0, trip the flag
            }
        }
    }
    
    char strConcatTo[20] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "zeros test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test scalar multiplication function: test that a matrix is equal to another matrix times a scalar value at every index
void test_scalar_multiplication(char str[], rc_matrix_t c, rc_matrix_t d, int m) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if (c.d[i][j]==d.d[i][j]*m) {
                //NOOP
            } else {
                flag = 1;
            }
        }
    }

    char strConcatTo[50] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix scalar multiplication test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test multiplication function: test that a matrix is sucessfully multiplied by another matrix
void test_multiplication(char str[], rc_matrix_t c, rc_matrix_t d, rc_matrix_t e) {
    int flag = 0; //test failed flag
    rc_matrix_t m;
    m = rc_matrix_empty();
    rc_matrix_zeros(&m, c.rows, d.cols);
    int inner = c.cols; //the shared inner value is both c.cols and d.rows

    //tripple nested common matrix multiplication function
    for (int i=0; (i<c.rows); i++) {
        for (int j=0; (j<d.cols); j++) {
            for (int k=0; (k<inner); k++) {
                m.d[i][j] = m.d[i][j] + c.d[i][k] * d.d[k][j];
            }
        }
    }

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<d.cols) && (flag==0); j++) {
            if ( e.d[i][j]==m.d[i][j] ) {
                //NOOP
            } else {
                flag = 1;
            }
        }
    }

    char strConcatTo[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix multiplication test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
    rc_matrix_free(&m); //free memory
}

//test addition function: test that a matrix is sucessfully added to another matrix
void test_addition(char str[], rc_matrix_t c, rc_matrix_t d, rc_matrix_t e) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if (c.d[i][j]+d.d[i][j] == e.d[i][j]) {
                //NOOP
            } else {
                flag = 1;
            }
        }
    }

    char strConcatTo[50] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix addition test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test subtraction function: test that a matrix is sucessfully subtracted from another matrix
void test_subtraction(char str[], rc_matrix_t c, rc_matrix_t d, rc_matrix_t e) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if (c.d[i][j]-d.d[i][j] == e.d[i][j]) {
                //NOOP
            } else {
                flag = 1;
            }
        }
    }

    char strConcatTo[50] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix subtraction test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test transpose function: test that a matrix is equal to another matrix when the rows and collumn indexes are flipped
void test_transpose(char str[], rc_matrix_t c, rc_matrix_t d) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<c.rows) && (flag==0); i++) {
        for (int j=0; (j<c.cols) && (flag==0); j++) {
            if (c.d[i][j]==d.d[j][i]) {
                //NOOP
            } else {
                flag = 1;
            }
        }
    }

    char strConcatTo[50] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix transpose test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//function that converts a row vector to a matrix
rc_matrix_t row_to_matrix(rc_vector_t v) {
    rc_matrix_t c = rc_matrix_empty();
    rc_matrix_zeros(&c, 1, v.len);
    for (int i=0; i<v.len; i++) {
        c.d[0][i] = v.d[i];
    }
    return c;
}

//function that converts a column vector to a matrix
rc_matrix_t col_to_matrix(rc_vector_t v) {
    rc_matrix_t c = rc_matrix_empty();
    rc_matrix_zeros(&c, v.len, 1);
    for (int i=0; i<v.len; i++) {
        c.d[i][0] = v.d[i];
    }
    return c;
}

//function that converts a one row or one column matrix to a row or column vector
rc_vector_t to_vector(rc_matrix_t c) {
    rc_vector_t v = rc_vector_empty();
    if (c.rows == 1) {
        rc_vector_zeros(&v, c.cols);
        for (int i=0; i<v.len; i++) {
            v.d[i] = c.d[0][i];
        }
    } else if (c.cols == 1) {
        rc_vector_zeros(&v, c.rows);
        for (int i=0; i<v.len; i++) {
            v.d[i] = c.d[i][0];
        }
    } else {
        printf("%d %d",c.rows,c.cols);
        printf("ERROR: MATRIX IS NOT A COLUMN OR ROW VECTOR");
    }

    return v;
}

//test row vector times matrix function: test that matrix multiplication works with a row vector and matrix
void test_row_vec_times_matrix(char str[], rc_matrix_t c, rc_vector_t v, rc_vector_t w) {
    rc_matrix_t d;
    d = rc_matrix_empty();
    d = row_to_matrix(v); //turn vector v into a one column matrix so that rc_matrix_multiply() can be used

    rc_vector_t y;
    y = rc_vector_empty();

    rc_matrix_t m;
    m = rc_matrix_empty();
    
    int flag = 0; //test failed flag

    rc_matrix_multiply(d, c, &m); //write multiplication result to matrix m
    
    y = to_vector(m); //write one row matrix m to vector y

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<w.len) && (flag==0); i++) {
        if ( y.d[i]==w.d[i] ) {
            //NOOP
        } else {
            flag = 1;
        }
    }

    char strConcatTo[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix row times matrix test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
    rc_matrix_free(&m);rc_matrix_free(&d);rc_vector_free(&y); //free memory
}

//test matrix times column vector function: test that matrix multiplication works with a rmatrix and column vector
void test_matrix_times_col_vec(char str[], rc_matrix_t c, rc_vector_t v, rc_vector_t w) {
    rc_matrix_t d;
    d = rc_matrix_empty();
    d = col_to_matrix(v); //turn vector v into a one column matrix so that rc_matrix_multiply() can be used

    rc_vector_t y;
    y = rc_vector_empty();

    rc_matrix_t m;
    m = rc_matrix_empty();
    
    int flag = 0; //test failed flag

    rc_matrix_multiply(c, d, &m); //write multiplication result to matrix m
    
    y = to_vector(m); //write column matrix m to vector y

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<w.len) && (flag==0); i++) {
        if ( y.d[i]==w.d[i] ) {
            //NOOP
        } else {
            flag = 1;
        }
    }

    char strConcatTo[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "matrix times col vector test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
    rc_matrix_free(&m);rc_matrix_free(&d);rc_vector_free(&y); //free memory
}

void test_outer_product(char str[], rc_vector_t v, rc_vector_t w, rc_matrix_t c) {
    int flag = 0; //test failed flag

    //double nested matrix index checker, break condition when flag tripped or no more indexes
    for (int i=0; (i<v.len) && (flag = 0); i++) {
		for (int j=0; (j<w.len) && (flag = 0); j++) {
			if ( c.d[i][j] == v.d[i]*w.d[j] ) {
                //NOOP
            } else {
                flag = 1;
            }
		}
	}

    char strConcatTo[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "outer product test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test determinant function: test that two numbers are within a set tolerance of one another
void test_determinant(char str[], double n1, double n2) {
    int flag = 0; //test failed flag

    if (fabs(n1 - n2) < tolerance) {
        //NOOP
    } else {
        flag = 1;
    }
	
    char strConcatTo[40] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "determinant test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}

//test symmetric function: test that the transopse is equal to all the indices of the original matrix
void test_symmetric(char str[], rc_matrix_t c) {
    int flag = 0; //test failed flag

    rc_matrix_t d = rc_matrix_empty();
    rc_matrix_transpose(c, &d);

    for (int i=0; (i<c.rows) && (flag = 0); i++) {
		for (int j=0; (j<c.rows) && (flag = 0); j++) {
			if ( c.d[i][j] == d.d[i][j] ) {
                //NOOP
            } else {
                flag = 1;
            }
		}
	}
	
    char strConcatTo[30] = ""; //must create a char array of adequate length to concatenate both strings to
    strcat(strConcatTo, "symmetric test ");
    strcat(strConcatTo, str);
    test(strConcatTo, flag==0); //test that the flag was not tripped
}







// main function, holds all tests
int main() {
    stdio_init_all(); //initialize the pico
    pause(); //first pause is ignored, put this here for a clean start

    goto rc_matrix_empty; //change this goto to a function name to skip to that test


    { //all tests
    


        rc_matrix_empty:hide_unused_error
        { //test the rc_matrix_t constructor method [ rc_matrix_empty ]
            pause();

            printf(CYN "\nTest the rc_matrix_t constructor method [ rc_matrix_empty ]:" RST);
            rc_matrix_t c;
            c = rc_matrix_empty();
            sleep_ms(200);

            test("rows test", c.rows==0); //assert that rc_matrix_empty initializes to "0" rows
            test("columns test", c.cols==0); //assert that rc_matrix_empty initializes to "0" cols
            test("initialized test", c.initialized==0); //assert that rc_matrix_empty initializes to "0" for the "initialized" field
            test("pointer test", c.d==NULL); //assert that rc_matrix_t rc_matrix_empty initializes to a null pointer
            rc_matrix_free(&c); //free memory
            sleep_ms(200);

            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_zeros:hide_unused_error
        { //test the rc_matric_t zeros matrix initializer method [ rc_matrix_zeros ]
            pause();

            printf(CYN "\nTest the zeros matrix initializer method [ rc_matrix_zeros ]:" RST);

            { //subtest 1: invalid input
                sleep_ms(200);

                printf(CYN "\n\tFirst subtest, invalid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_zeros(&c, 1, 1)==-1); //assert that rc_matrix_t errors when given a null pointer
                test("correct values test (null ptr)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory

                test("invalid dimensions test (0x0)", rc_matrix_zeros(&c, 0, 0)==-1); //assert that rc_matrix_t errors with invalid dimensions
                test("correct values test (0x0)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory

                test("invalid dimensions test (1x0)", rc_matrix_zeros(&c, 1, 0)==-1); //assert that rc_matrix_t errors with invalid dimensions
                test("correct values test (1x0)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory

                test("invalid dimensions test (0x5)", rc_matrix_zeros(&c, 0, 5)==-1); //assert that rc_matrix_t errors with invalid dimensions
                test("correct values test (0x5)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory

                test("invalid dimensions test (-2x3)", rc_matrix_zeros(&c, -2, 3)==-1); //assert that rc_matrix_t errors with invalid dimensions
                test("correct values test (-2x3)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory

                test("invalid dimensions test (-1x-5)", rc_matrix_zeros(&c, -1, -5)==-1); //assert that rc_matrix_t errors with invalid dimensions
                test("correct values test (-1x-5)", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t has not changed values
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: lower dimensions
                pause();

                printf(CYN "\n\tSecond subtest, normal dimensions:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                test("valid dimensions test (1x1)", rc_matrix_zeros(&c, 1, 1)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (1x1)", c.rows==1&&c.cols==1&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (1x1)", rc_matrix_zeros(&c, 1, 1)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (1x1)", c.rows==1&&c.cols==1&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (5x5)", rc_matrix_zeros(&c, 5, 5)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (5x5)", c.rows==5&&c.cols==5&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (10x1)", rc_matrix_zeros(&c, 10, 1)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (10x1)", c.rows==10&&c.cols==1&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (2x15)", rc_matrix_zeros(&c, 2, 15)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (2x15)", c.rows==2&&c.cols==15&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 3: higher dimesnsions
                pause();

                printf(CYN "\n\tThird subtest, higher dimensions:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                test("valid dimensions test (1000x1)", rc_matrix_zeros(&c, 1000, 1)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (1000x1)", c.rows==1000&&c.cols==1&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (5x1050)", rc_matrix_zeros(&c, 5, 1050)==0); //assert that rc_matrix_t is successfull with valid dimensions
                test("correct values test (5x1050)", c.rows==5&&c.cols==1050&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory

                test("valid dimensions test (127x127)", rc_matrix_zeros(&c, 127, 127)==0); //assert that rc_matrix_t is sucessful at the highest dimension possible on pico
                test("correct values test (127x127)", c.rows==127&&c.cols==127&&c.initialized==1&&c.d!=NULL); //assert that rc_matrix_t is given correct values
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_free:hide_unused_error
        { //test the rc_matrix_t memory "delete" method [ rc_matrix_free ]
            pause();

            printf(CYN "\nTest the memory delete method [ rc_matrix_free ]" RST);
            rc_matrix_t c;
            c = rc_matrix_empty();
            sleep_ms(200);
                
            c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
            test("null pointer test", rc_matrix_free(&c)==-1); //assert that rc_matrix_t errors when a null pointer is provided
            rc_matrix_free(&c); //free memory

            rc_matrix_zeros(&c, 1, 4);
            test("valid matrix error test", rc_matrix_free(&c)==0); //assert that rc_matrix_t is sucessful when a valid matrix is provided
            test("valid matrix correct values test", c.rows==0&&c.cols==0&&c.initialized==0&&c.d==NULL); //assert that rc_matrix_t is reverted to a rc_matrix_empty() matrix
            rc_matrix_free(&c); //free memory
            sleep_ms(200);

            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }


        
        rc_matrix_alloc:hide_unused_error
        { //test the rc_matrix_t memory allocation method [ rc_matrix_alloc ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t memory allocation method [ rc_matrix_alloc ]" RST);

            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_alloc(&c,1,1)==-1); //assert that rc_matrix_t errors when a null pointer matrix is provided
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                test("allocation to invalid dim test (0x3)", rc_matrix_alloc(&c,0,3)==-1); //assert that rc_matrix_t errors with invalid allocation dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                test("allocation to invalid dim test (-1x1)", rc_matrix_alloc(&c,-1,1)==-1); //assert that rc_matrix_t errors with invalid allocation dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                test("allocation to invalid dim test (-1x-4)", rc_matrix_alloc(&c,-1,-4)==-1); //assert that rc_matrix_t errors with invalid allocation dimensions
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                rc_matrix_zeros(&c, 1, 1);
                test("allocation to valid dim test (1x1->1x1)", rc_matrix_alloc(&c,1,1)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 2);
                test("allocation to valid dim test (1x2->1x1)", rc_matrix_alloc(&c,1,1)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when shrinking matrix columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 4, 1);
                test("allocation to valid dim test (4x1->1x1)", rc_matrix_alloc(&c,1,1)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when shrinking matrix rows
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 2, 2);
                test("allocation to valid dim test (2x2->1x1)", rc_matrix_alloc(&c,1,1)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when shrinking matrix rows and columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                test("allocation to valid dim test (1x1->3x1)", rc_matrix_alloc(&c,3,1)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when growing rows
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 3, 2);
                test("allocation to valid dim test (3x2->3x5)", rc_matrix_alloc(&c,3,5)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when growing columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 3, 6);
                test("allocation to valid dim test (3x6->5x7)", rc_matrix_alloc(&c,5,7)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when growing rows and columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 6);
                test("allocation to valid dim test (1x6->7x2)", rc_matrix_alloc(&c,7,2)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when growing rows and shrinking columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 8, 1);
                test("allocation to valid dim test (8x1->3x12)", rc_matrix_alloc(&c,3,12)==0); //assert that rc_matrix_t is sucessful with valid allocation dimensions when shrinking rows and growing columns
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                test("allocation to valid dim test (1x1->127x127)", rc_matrix_alloc(&c,127,127)==0); //assert that rc_matrix_t is sucessful at allocating to the highest dimension possible on pico
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }
            
            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_identity:hide_unused_error
        { //test the rc_matrix_t identity matrix method [ rc_matrix_identity ]
            pause();

            printf(CYN "\nTest the rc_matrix_t identity matrix method [ rc_matrix_identity ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_identity(&c, 1) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c); //free memory

                test("invalid dimension test (0)", rc_matrix_identity(&c, 0) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension
                rc_matrix_free(&c); //free memory

                test("invalid dimension test (-3)", rc_matrix_identity(&c, -3) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension
                rc_matrix_free(&c); //free memory

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_identity(&c, 1);
                test_identity("(1)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimension
                rc_matrix_free(&c); //free memory

                rc_matrix_identity(&c, 3);
                test_identity("(3)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimension
                rc_matrix_free(&c); //free memory

                rc_matrix_identity(&c, 8);
                test_identity("(8)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimension
                rc_matrix_free(&c); //free memory

                rc_matrix_identity(&c, 21);
                test_identity("(21)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimension
                rc_matrix_free(&c); //free memory
                sleep_ms(200);
               
                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_random:hide_unused_error
        { //test the rc_matrix_t random matrix method [ rc_matrix_random ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t random matrix method [ rc_matrix_random ]" RST);
            
            { //subtest: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
                rc_matrix_t c;
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_random(&c, 1, 1) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c); //free memory

                test("invalid dimension test (-2x1)", rc_matrix_random(&c, -2, 1) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension
                rc_matrix_free(&c); //free memory

                test("invalid dimension test (4x0)", rc_matrix_random(&c, 4, 0) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension
                rc_matrix_free(&c); //free memory

                test("invalid dimension test (-4x-2)", rc_matrix_random(&c, -4, -2) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension
                rc_matrix_free(&c); //free memory
                
                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
                rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                test_random("(1x1)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 2, 5);
                test_random("(2x5)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 16, 3);
                test_random("(16x3)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimensions
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 2, 113);
                test_random("(2x113)", c); //assert that rc_matrix_t is sucessful when provided with a valid dimensions
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }
    
        

        rc_matrix_diagonal:hide_unused_error
        { //test the rc_matrix_t diagonal insert vector to matrix method [ rc_matrix_diagonal ]
            pause();

            printf(CYN "\nTest the rc_matrix_t diagonal insert vector to matrix method [ rc_matrix_diagonal ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                sleep_ms(200);
		        
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                rc_vector_zeros(&v, 1);
                test("test null pointer (matrix)", rc_matrix_diagonal(&c, v) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 1, 1);
                v.len=1;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test null pointer (vector)", rc_matrix_diagonal(&c, v) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                v.len=1;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test null pointer (both)", rc_matrix_diagonal(&c, v) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the initialized is 0
                v.len=1;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test non-initialized matrix", rc_matrix_diagonal(&c, v) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 2, 2);
                v.len=-2;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test invalid dims (2x2 insert -2)", rc_matrix_diagonal(&c, v) == -1);  //assert that rc_matrix_t errors when provided with an invalid dimenension vector
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 5, 5);
                v.len=0;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test invalid dims (5x5 insert 0)", rc_matrix_diagonal(&c, v) == -1);  //assert that rc_matrix_t errors when provided with an invalid dimenension vector
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                c.rows=0;c.cols=0;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                v.len=0;v.d=NULL; //set the values of the vector such that the dimensions are valid but the pointer is null
                test("test invalid dims (0x0 insert 0)", rc_matrix_diagonal(&c, v) == -1);  //assert that rc_matrix_t errors when provided with an invalid dimension matrix and invalid dimenension vector
                rc_matrix_free(&c);rc_vector_free(&v); //free memory
                sleep_ms(200);
		
		        printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
		    }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                sleep_ms(200);
                
                rc_matrix_zeros(&c, 2, 2);
                rc_vector_random(&v, 2);
                test("general error test (2->2x2)", rc_matrix_diagonal(&c, v)==0);
                test_diagonal("(2->2x2)", c); //assert that rc_matrix_t is sucessful when provided with a valid vector and a valid matrix
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 4, 4);
                rc_vector_random(&v, 4);
                test("general error test (4->4x4)", rc_matrix_diagonal(&c, v)==0);
                test_diagonal("(4->4x4)", c); //assert that rc_matrix_t is sucessful when provided with a valid vector and a valid matrix
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 2, 3);
                rc_vector_random(&v, 1);
                test("general error test (1->2x3)", rc_matrix_diagonal(&c, v)==0);
                test_diagonal("(1->2x3)", c); //assert that rc_matrix_t is sucessful when provided with a smaller vector and a larger matrix
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                rc_matrix_zeros(&c, 5, 9);
                rc_vector_random(&v, 12);
                test("general error test (12->5x9)", rc_matrix_diagonal(&c, v)==0);
                test_diagonal("(12->5x9)", c); //assert that rc_matrix_t is sucessful when provided with a larger vector and a smaller matrix
                rc_matrix_free(&c);rc_vector_free(&v); //free memory

                c.rows=-8;c.cols=2; //set the values of the matrix such that the dimensions are invalid
                rc_vector_random(&v, 3);
                test("general error test (3->-8x2)", rc_matrix_diagonal(&c, v)==0);
                test_diagonal("(3->-8x2)", c); //assert that rc_matrix_t is sucessful when provided with an invailid dimension matrix and a vaild vector
                rc_matrix_free(&c);rc_vector_free(&v); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }
            
            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_duplicate:hide_unused_error
        { //test the rc_matrix_t duplication method [ rc_matrix_duplicate ]
            pause();

            printf(CYN "\nTest the rc_matrix_t duplication method [ rc_matrix_duplicate ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_duplicate(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=-1;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test", rc_matrix_duplicate(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                test("self reference error test", rc_matrix_duplicate(c, &c) == 0); //assert that rc_matrix_t does not error when provided with itself
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 2);
                test("general error test (2x2)", rc_matrix_duplicate(c, &d) == 0); //assert that no errors occur with a valid vector and matrix
                test_compare("(2x2)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 4, 6);
                test("general error test (4x6)", rc_matrix_duplicate(c, &d) == 0); //assert that no errors occur with a valid vector and matrix
                test_compare("(4x6)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 7);
                test("general error test (1x7)", rc_matrix_duplicate(c, &d) == 0); //assert that no errors occur with a valid vector and matrix
                test_compare("(1x7)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);
                
                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_zero_out:hide_unused_error
        { //test the rc_matrix_t zero out method [ rc_matrix_zero_out ]
            pause();

            printf(CYN "\nTest the rc_matrix_t zero out method [ rc_matrix_zero_out ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_zero_out(&c) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c); //free memory

                c.rows=-1;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test", rc_matrix_zero_out(&c) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                rc_matrix_random(&c, 2, 2);
                test("general error test (2x2)", rc_matrix_zero_out(&c) == 0); //assert that rc_matrix_t is sucessful when provided a valid matrix
                test_zeros("(2x2)", c);
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 6, 6);
                test("general error test (6x6)", rc_matrix_zero_out(&c) == 0); //assert that rc_matrix_t is sucessful when provided a valid matrix
                test_zeros("(6x6)", c);
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 2, 3);
                test("general error test (2x3)", rc_matrix_zero_out(&c) == 0); //assert that rc_matrix_t is sucessful when provided a valid matrix
                test_zeros("(2x3)", c);
                rc_matrix_free(&c); //free memory
                sleep_ms(200);
                
                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_times_scalar:hide_unused_error
        { //test the rc_matrix_t matrix times scalar method [ rc_matrix_times_scalar ]
            pause();

            printf(CYN "\nTest the rc_matrix_t matrix times scalar method [ rc_matrix_times_scalar ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test", rc_matrix_times_scalar(&c, 2) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c); //free memory

                c.rows=-1;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test", rc_matrix_times_scalar(&c, 2) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                int m;
                sleep_ms(200);
                
                rc_matrix_random(&c, 2, 2);
                rc_matrix_duplicate(c, &d);
                m = 7;
                test("general error test (2x2*7)", rc_matrix_times_scalar(&c, m) == 0); //assert that no errors occur with a valid vector and matrix
                test_scalar_multiplication("(2x2*7)", c, d, m);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 5, 3);
                rc_matrix_duplicate(c, &d);
                m = 3;
                test("general error test (5x3*3)", rc_matrix_times_scalar(&c, m) == 0); //assert that no errors occur with a valid vector and matrix
                test_scalar_multiplication("(5x3*3)", c, d, m);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 10, 6);
                rc_matrix_duplicate(c, &d);
                m = -9;
                test("general error test (10x6*-9)", rc_matrix_times_scalar(&c, m) == 0); //assert that no errors occur with a valid vector and matrix
                test_scalar_multiplication("(10x6*-9)", c, d, m);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 8);
                rc_matrix_duplicate(c, &d);
                m = 0;
                test("general error test (2x8*0)", rc_matrix_times_scalar(&c, m) == 0); //assert that no errors occur with a valid vector and matrix
                test_scalar_multiplication("(2x8*0)", c, d, m);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                sleep_ms(200);
                
                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }


        rc_matrix_multiply:hide_unused_error
        { //test the rc_matrix_t matrix multiplication method [ rc_matrix_multiply ]
            pause();

            printf(CYN "\nTest the rc_matrix_t matrix times matrix method [ rc_matrix_multiply ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;c.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                d.rows=1;d.cols=1;c.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 5, 1);
                test("invalid dimension error test (-1x5*5x1)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 2, 4);
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x4*4x-5)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                c.rows=-5;c.cols=4; //set the values of the matrix such that the dimensions are invalid
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-5x4*4x-5)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0*0x5)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 5, 9);
                test("invalid multiplication dimensions test (1x6*5x9)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 5, 1);
                rc_matrix_random(&d, 5, 1);
                test("invalid multiplication dimensions error test (5x1*5x1)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 5, 7);
                rc_matrix_random(&d, 9, 5);
                test("invalid multiplication dimensions error test (5x7*9x5)", rc_matrix_multiply(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                test("general error test (1x1*1x1)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(1x1*1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                test("general error test (3x3*3x3)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(3x3*3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 7);
                rc_matrix_random(&d, 7, 3);
                test("general error test (3x7*7x3)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies
                test_multiplication("(3x7*7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 2, 9);
                test("general error test (9x2*2x9)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies of zeros
                test_multiplication("(9x2*2x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 6, 11);
                rc_matrix_random(&d, 11, 6);
                test("general error test (6x11*11x6)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid non-similar dimensional matricies
                test_multiplication("(6x11*11x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 4, 3);
                rc_matrix_random(&d, 3, 6);
                test("general error test (4x3*3x6)", rc_matrix_multiply(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two non-similar valid dimensional matricies
                test_multiplication("(4x3*3x6)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_left_multiply_inplace:hide_unused_error
        { //test the rc_matrix_t matrix left multiplication (low mem usage) method [ rc_matrix_left_multiply_inplace ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix left multiplication (low mem usage) method [ rc_matrix_left_multiply_inplace ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                d.rows=1;d.cols=1;d.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 5, 1);
                test("invalid dimension error test (-1x5*5x1)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 2, 4);
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x4*4x-5)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=-5;c.cols=4; //set the values of the matrix such that the dimensions are invalid
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-5x4*4x-5)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0*0x5)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 5, 9);
                test("invalid multiplication dimensions test (1x6*5x9)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 5, 1);
                rc_matrix_random(&d, 5, 1);
                test("invalid multiplication dimensions error test (5x1*5x1)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 5, 7);
                rc_matrix_random(&d, 9, 5);
                test("invalid multiplication dimensions error test (5x7*9x5)", rc_matrix_left_multiply_inplace(c, &d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                rc_matrix_duplicate(d, &e);
                test("general error test (1x1*1x1)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(1x1*1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                rc_matrix_duplicate(d, &e);
                test("general error test (3x3*3x3)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(3x3*3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 7);
                rc_matrix_random(&d, 7, 3);
                rc_matrix_duplicate(d, &e);
                test("general error test (3x7*7x3)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies
                test_multiplication("(3x7*7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 2, 9);
                rc_matrix_duplicate(d, &e);
                test("general error test (9x2*2x9)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies of zeros
                test_multiplication("(9x2*2x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 6, 11);
                rc_matrix_random(&d, 11, 6);
                rc_matrix_duplicate(d, &e);
                test("general error test (6x11*11x6)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid non-similar dimensional matricies
                test_multiplication("(6x11*11x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 4, 3);
                rc_matrix_random(&d, 3, 6);
                rc_matrix_duplicate(d, &e);
                test("general error test (4x3*3x6)", rc_matrix_left_multiply_inplace(c, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two non-similar valid dimensional matricies
                test_multiplication("(4x3*3x6)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_right_multiply_inplace:hide_unused_error
        { //test the rc_matrix_t matrix right multiplication (low mem usage) method [ rc_matrix_right_multiply_inplace ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix right multiplication (low mem usage) method [ rc_matrix_right_multiply_inplace ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=1;c.cols=1;c.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                d.rows=1;d.cols=1;d.d=NULL; //set the values of the matrix such that the dimensions are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 5, 1);
                test("invalid dimension error test (-1x5*5x1)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 2, 4);
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x4*4x-5)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=-5;c.cols=4; //set the values of the matrix such that the dimensions are invalid
                d.rows=4;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-5x4*4x-5)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0*0x5)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 5, 9);
                test("invalid multiplication dimensions test (1x6*5x9)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 5, 1);
                rc_matrix_random(&d, 5, 1);
                test("invalid multiplication dimensions error test (5x1*5x1)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 5, 7);
                rc_matrix_random(&d, 9, 5);
                test("invalid multiplication dimensions error test (5x7*9x5)", rc_matrix_right_multiply_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix multiplication
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                rc_matrix_duplicate(c, &e);
                test("general error test (1x1*1x1)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(1x1*1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (3x3*3x3)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_multiplication("(3x3*3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 7);
                rc_matrix_random(&d, 7, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (3x7*7x3)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies
                test_multiplication("(3x7*7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 2, 9);
                rc_matrix_duplicate(c, &e);
                test("general error test (9x2*2x9)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid similar dimensional matricies of zeros
                test_multiplication("(9x2*2x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 6, 11);
                rc_matrix_random(&d, 11, 6);
                rc_matrix_duplicate(c, &e);
                test("general error test (6x11*11x6)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid non-similar dimensional matricies
                test_multiplication("(6x11*11x9)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 4, 3);
                rc_matrix_random(&d, 3, 6);
                rc_matrix_duplicate(c, &e);
                test("general error test (4x3*3x6)", rc_matrix_right_multiply_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two non-similar valid dimensional matricies
                test_multiplication("(4x3*3x6)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_add:hide_unused_error
        { //test the rc_matrix_t matrix addition method [ rc_matrix_add ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix addition method [ rc_matrix_add ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                
                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 1, 5);
                test("invalid dimension error test (-1x5+1x5)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=-5; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x-5+3x-5)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                d.rows=3;d.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x0+3x0)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                d.rows=5;d.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0+0x5)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions test (1x6+1x1)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions error test (2x1+1x1)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 2);
                test("invalid addition dimensions error test (2x1+1x2)", rc_matrix_add(c, d, &e) == -1); //assert that rc_matrix_t errors when provided with transposed dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                test("general error test (1x1+1x1)", rc_matrix_add(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_addition("(1x1+1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                test("general error test (3x3+3x3)", rc_matrix_add(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_addition("(3x3+3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 7, 3);
                rc_matrix_random(&d, 7, 3);
                test("general error test (7x3+7x3)", rc_matrix_add(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies
                test_addition("(7x3+7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 9, 2);
                test("general error test (9x2+9x2)", rc_matrix_add(c, d, &e) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies of zeros
                test_addition("(9x2+9x2)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_add_inplace:hide_unused_error
        { //test the rc_matrix_t matrix addition (low mem usage) method [ rc_matrix_add_inplace ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix addition (low mem usage) method [ rc_matrix_add_inplace ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                
                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 1, 5);
                test("invalid dimension error test (-1x5+1x5)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=-5; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x-5+3x-5)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x0+3x0)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0+0x5)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions test (1x6+1x1)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions error test (2x1+1x1)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 2);
                test("invalid addition dimensions error test (2x1+1x2)", rc_matrix_add_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with transposed dimensions for matrix addition
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                rc_matrix_duplicate(c, &e);
                test("general error test (1x1+1x1)", rc_matrix_add_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_addition("(1x1+1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (3x3+3x3)", rc_matrix_add_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_addition("(3x3+3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 7, 3);
                rc_matrix_random(&d, 7, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (7x3+7x3)", rc_matrix_add_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies
                test_addition("(7x3+7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 9, 2);
                rc_matrix_duplicate(c, &e);
                test("general error test (9x2+9x2)", rc_matrix_add_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies of zeros
                test_addition("(9x2+9x2)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_subtract_inplace:hide_unused_error
        { //test the rc_matrix_t matrix subtraction (low mem usage) method [ rc_matrix_add_inplace ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix subtraction (low mem usage) method [ rc_matrix_subtract_inplace ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                rc_matrix_zeros(&c, 1, 1);

                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (right matrix)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                d.rows=1;d.cols=1;d.initialized=1;d.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (both matricies)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (right matrix)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (right)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                rc_matrix_zeros(&d, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                d.rows=1;d.cols=1;d.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                test("non-initialized test (both matricies)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (both)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                
                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                rc_matrix_zeros(&d, 1, 5);
                test("invalid dimension error test (-1x5-1x5)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=-5; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x-5-3x-5)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=3;d.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x0-3x0)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                d.rows=0;d.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0-0x5)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 1, 6);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions test (1x6-1x1)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix subtraction
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 1);
                test("invalid addition dimensions error test (2x1-1x1)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions for matrix subtraction
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_matrix_random(&d, 1, 2);
                test("invalid addition dimensions error test (2x1-1x2)", rc_matrix_subtract_inplace(&c, d) == -1); //assert that rc_matrix_t errors when provided with transposed dimensions for matrix subtraction
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            
            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                rc_matrix_t e;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                e = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_random(&d, 1, 1);
                rc_matrix_duplicate(c, &e);
                test("general error test (1x1-1x1)", rc_matrix_subtract_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_subtraction("(1x1-1x1)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_random(&d, 3, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (3x3-3x3)", rc_matrix_subtract_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_subtraction("(3x3-3x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_random(&c, 7, 3);
                rc_matrix_random(&d, 7, 3);
                rc_matrix_duplicate(c, &e);
                test("general error test (7x3-7x3)", rc_matrix_subtract_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies
                test_subtraction("(7x3-7x3)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_zeros(&d, 9, 2);
                rc_matrix_duplicate(c, &e);
                test("general error test (9x2-9x2)", rc_matrix_subtract_inplace(&e, d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies of zeros
                test_subtraction("(9x2-9x2)", c, d, e);
                rc_matrix_free(&c);rc_matrix_free(&d);rc_matrix_free(&e); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }



        rc_matrix_transpose:hide_unused_error
        { //test the rc_matrix_t matrix transpose method [ rc_matrix_transpose ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix transpose method [ rc_matrix_transpose ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_matrix_zeros(&d, 1, 1);
                test("null pointer test (left matrix)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the dimensions are valid but the matrix is uninitialized
                rc_matrix_zeros(&d, 1, 1);
                test("non-initialized test (left matrix)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with a null pointer (left)
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                
                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-1x5)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x-5)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=3;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x0)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0)", rc_matrix_transpose(c, &d) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                test("general error test (1x1)", rc_matrix_transpose(c, &d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_transpose("(1x1)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 3, 3);
                test("general error test (3x3)", rc_matrix_transpose(c, &d) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_transpose("(3x3)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 7, 3);
                test("general error test (7x3)", rc_matrix_transpose(c, &d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies
                test_transpose("(7x3)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 9, 2);
                test("general error test (9x2)", rc_matrix_transpose(c, &d) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies of zeros
                test_transpose("(9x2)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }


        rc_matrix_transpose_inplace:hide_unused_error
        { //test the rc_matrix_t matrix transpose (low mem usage) method [ rc_matrix_transpose_inplace ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix transpose (low mem usage) method [ rc_matrix_transpose_inplace ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test", rc_matrix_transpose_inplace(&c) == -1); //assert that rc_matrix_t errors when provided with a null pointer
                rc_matrix_free(&c); //free memory
                
                c.rows=-1;c.cols=5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-1x5)", rc_matrix_transpose_inplace(&c) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the first field
                rc_matrix_free(&c); //free memory

                c.rows=3;c.cols=-5; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x-5)", rc_matrix_transpose_inplace(&c) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in the second field
                rc_matrix_free(&c); //free memory

                c.rows=3;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (3x0)", rc_matrix_transpose_inplace(&c) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c); //free memory

                c.rows=5;c.cols=0; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x0)", rc_matrix_transpose_inplace(&c) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix in both fields
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_matrix_t d;
                c = rc_matrix_empty();
                d = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_matrix_duplicate(c, &d);
                test("general error test (1x1)", rc_matrix_transpose_inplace(&c) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_transpose("(1x1)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 3, 3);
                rc_matrix_duplicate(c, &d);
                test("general error test (3x3)", rc_matrix_transpose_inplace(&c) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_transpose("(3x3)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_random(&c, 7, 3);
                rc_matrix_duplicate(c, &d);
                test("general error test (7x3)", rc_matrix_transpose_inplace(&c) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies
                test_transpose("(7x3)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory

                rc_matrix_zeros(&c, 9, 2);
                rc_matrix_duplicate(c, &d);
                test("general error test (9x2)", rc_matrix_transpose_inplace(&c) == 0); //assert that rc_matrix_t is sucessful when provided with two valid same dimensional matricies of zeros
                test_transpose("(9x2)", c, d);
                rc_matrix_free(&c);rc_matrix_free(&d); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_row_vec_times_matrix:hide_unused_error
        { //test the rc_matrix_t matrix row vector times matrix method [ rc_matrix_row_vec_times_matrix ]
            pause();

            printf(CYN "\nTest the rc_matrix_t matrix times row vector method [ rc_matrix_row_vec_times_matrix ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_vector_zeros(&v, 1);
                rc_vector_zeros(&w, 1);
                test("null pointer test (matrix)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with a null pointer matrix
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 1, 1); //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                v.len=1;v.initialized=1;v.d=NULL;
                rc_vector_zeros(&w, 1);
                test("null pointer test (vector)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=-1;c.cols=5;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 5);
                test("invalid dimension error test (5*-1x5)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=3;c.cols=-5;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 1);
                v.len=-5;v.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (-5*3x-5)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension in both fields
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=3;c.cols=0;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 1);
                v.len=-5;c.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (0*3x0)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension in both fields
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 5, 5);
                rc_vector_zeros(&v, 1);
                v.len=0;c.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (0*5x5)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 5, 5);
                rc_vector_zeros(&v, 1);
                v.len=-5;v.initialized=1; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (-5*5x5)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 4, 2);
                rc_vector_zeros(&v, 5);
                test("invalid dimension error test (5*4x2)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimensions for matrix vector multiplication
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 2, 5);
                rc_vector_zeros(&v, 5);
                test("invalid dimension error test (5*2x5)", rc_matrix_row_vec_times_matrix(v, c, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimensions for matrix vector multiplication
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_vector_random(&v, 1);
                test("general error test (1*1x1)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_row_vec_times_matrix("(1*1x1)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 2);
                rc_vector_random(&v, 2);
                test("general error test (2*2x2)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided a square matrix and a valid dimensional vector
                test_row_vec_times_matrix("(2*2x2)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 3);
                rc_vector_random(&v, 2);
                test("general error test (2*2x3)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(2*2x3)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 4, 7);
                rc_vector_random(&v, 4);
                test("general error test (4*4x7)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(4*4x7)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_vector_random(&v, 2);
                test("general error test (2*2x1)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(2*2x1)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_random(&c, 6, 3);
                rc_vector_random(&v, 6);
                test("general error test (6*6x3)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(6*6x3)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 6, 4);
                rc_vector_random(&v, 6);
                test("general error test (6*6x4)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(6*6x4)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 6, 9);
                rc_vector_random(&v, 6);
                test("general error test (6*6x9)", rc_matrix_row_vec_times_matrix(v, c, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_row_vec_times_matrix("(6*6x9)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }
        
            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        


        rc_matrix_times_col_vec:hide_unused_error
        { //test the rc_matrix_t matrix times col vector method [ rc_matrix_times_col_vec ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix times col vector method [ rc_matrix_times_col_vec ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                rc_vector_zeros(&v, 1);
                rc_vector_zeros(&w, 1);
                test("null pointer test (matrix)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with a null pointer matrix
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 1, 1); //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                v.len=1;v.initialized=1;v.d=NULL;
                rc_vector_zeros(&w, 1);
                test("null pointer test (vector)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=-1;c.cols=5;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 5);
                test("invalid dimension error test (-1x5*5)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension matrix
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=3;c.cols=5;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 1);
                v.len=-5;v.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (3x5*-5)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension in both fields
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=3;c.cols=0;c.initialized=1; //set the values of the matrix such that the dimensions are invalid
                rc_vector_zeros(&v, 1);
                v.len=-5;c.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (3x0*0)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension in both fields
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 5, 5);
                rc_vector_zeros(&v, 1);
                v.len=0;c.initialized=1; //set the values of the vector such that the dimensions are invalid
                test("invalid dimension error test (5x5*0)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 5, 5);
                rc_vector_zeros(&v, 1);
                v.len=-5;v.initialized=1; //set the values of the matrix such that the dimensions are invalid
                test("invalid dimension error test (5x5*-5)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimension vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_zeros(&c, 4, 2);
                rc_vector_zeros(&v, 5);
                test("invalid dimension error test (4x2*5)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimensions for matrix vector multiplication
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_zeros(&c, 2, 5);
                rc_vector_zeros(&v, 2);
                test("invalid dimension error test (2x5*2)", rc_matrix_times_col_vec(c, v, &w) == -1); //assert that rc_matrix_t errors when provided with an invalid dimensions for matrix vector multiplication
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                rc_vector_random(&v, 1);
                test("general error test (1x1*1)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with two square matricies
                test_matrix_times_col_vec("(1x1*1)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 2);
                rc_vector_random(&v, 2);
                test("general error test (2x2*2)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided a square matrix and a valid dimensional vector
                test_matrix_times_col_vec("(2x2*2)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 3);
                rc_vector_random(&v, 3);
                test("general error test (2x3*3)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(2x3*3)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 4, 7);
                rc_vector_random(&v, 7);
                test("general error test (4x7*7)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(4x7*7)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 2, 1);
                rc_vector_random(&v, 1);
                test("general error test (2x1*1)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(2x1*1)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                
                rc_matrix_random(&c, 6, 3);
                rc_vector_random(&v, 3);
                test("general error test (6x3*3)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(6x3*3)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 6, 4);
                rc_vector_random(&v, 4);
                test("general error test (6x4*4)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(6x4*4)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_matrix_random(&c, 6, 9);
                rc_vector_random(&v, 9);
                test("general error test (6x9*9)", rc_matrix_times_col_vec(c, v, &w) == 0); //assert that rc_matrix_t is sucessful when provided with similar dimensions
                test_matrix_times_col_vec("(6x9*9)", c, v, w);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);

        }



        rc_matrix_outer_product:hide_unused_error
        { //test the rc_matrix_t matrix outer product method [ rc_matrix_outer_product ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix outer product method [ rc_matrix_outer_product ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);
                
                v.len=1;v.initialized=1;v.d=NULL; //set the values of the first vector such that the dimensions and initialized are valid but the pointer is null
                rc_vector_zeros(&w, 1);
                test("null pointer test (first vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                w.len=1;w.initialized=1;w.d=NULL; //set the values of the second vector such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (second vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                v.len=1;v.initialized=1;v.d=NULL; //set the values of the first vector such that the dimensions and initialized are valid but the pointer is null
                w.len=1;w.initialized=1;w.d=NULL; //set the values of the second vector such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test (both vecs)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                v.len=1;v.initialized=0; //set the values of the first vector such that the dimensions are valid but the initialized is 0
                rc_vector_zeros(&w, 1);
                test("initialized test (first vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                rc_vector_zeros(&w, 1);
                w.len=1;w.initialized=0; //set the values of the second vector such that the dimensions are valid but the initialized is 0
                test("initialized test (second vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
            
                rc_vector_zeros(&v, 1);
                v.len=1;v.initialized=0; //set the values of the first vector such that the dimensions are valid but the initialized is 0
                rc_vector_zeros(&w, 1);
                w.len=1;w.initialized=0; //set the values of the second vector such that the dimensions are valid but the initialized is 0
                test("initialized test (both vecs)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                rc_vector_zeros(&w, 1);
                w.len=-1;w.initialized=1; //set the values of the second vector such that the dimensions are invalid
                test("invalid dim test (first vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                v.len=-1;v.initialized=1; //set the values of the first vector such that the dimensions are invalid
                rc_vector_zeros(&w, 1);
                test("invalid dim test (second vec)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_zeros(&v, 1);
                v.len=-1;v.initialized=1; //set the values of the first vector such that the dimensions are invalid
                rc_vector_zeros(&w, 1);
                w.len=-1;w.initialized=1; //set the values of the second vector such that the dimensions are invalid
                test("invalid dim test (both vecs)", rc_matrix_outer_product(v, w, &c) == -1); //assert that rc_matrix_t errors when provided with invalid dimensions
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                rc_vector_t v;
                rc_vector_t w;
                c = rc_matrix_empty();
                v = rc_vector_empty();
                w = rc_vector_empty();
                sleep_ms(200);

                rc_vector_random(&v, 1);
                rc_vector_random(&w, 1);
                test("general error test (1*1)", rc_matrix_outer_product(v, w, &c) == 0); //assert that rc_matrix_t is sucessful when provided with two square valid vectors
                test_outer_product("(1*1)", v, w, c);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_random(&v, 2);
                rc_vector_random(&w, 2);
                test("general error test (2*2)", rc_matrix_outer_product(v, w, &c) == 0); //assert that rc_matrix_t is sucessful when provided with two square valid vectors
                test_outer_product("(2*2)", v, w, c);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_random(&v, 3);
                rc_vector_random(&w, 6);
                test("general error test (3*6)", rc_matrix_outer_product(v, w, &c) == 0); //assert that rc_matrix_t is sucessful when provided with two square valid vectors
                test_outer_product("(3*6)", v, w, c);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory

                rc_vector_random(&v, 3);
                rc_vector_random(&w, 1);
                test("general error test (3*1)", rc_matrix_outer_product(v, w, &c) == 0); //assert that rc_matrix_t is sucessful when provided with two square valid vectors
                test_outer_product("(3*1)", v, w, c);
                rc_matrix_free(&c);rc_vector_free(&v);rc_vector_free(&w); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }
        
            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
            
        }

        

        rc_matrix_determinant:hide_unused_error
        { //test the rc_matrix_t matrix determinent method [ rc_matrix_determinant ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix determinent method [ rc_matrix_determinant ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the initialized is 0
                test("uninitialized test", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with an uninitialized matrix
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=-1;c.cols=-1; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (-1x-1)", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with invalid dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=0;c.cols=0; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test 0x0)", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with invalid dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=6;c.cols=-1; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (6x-1)", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with  invalid, non square dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=7;c.cols=3; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (7x3)", rc_matrix_determinant(c) == -1); //assert that rc_matrix_t errors when provided with non square dims
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_zeros(&c, 1, 1);
                c.d[0][0] = 3;
                test("general error test (1x1)", rc_matrix_determinant(c) != -1.0); //assert that rc_matrix_t is sucessful when provided with valid input
                test_determinant("(1x1)", rc_matrix_determinant(c), 3);
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 2, 2);
                c.d[0][0] = 3;
                c.d[0][1] = 1;
                c.d[1][0] = 6;
                c.d[1][1] = 5;
                test("general error test (2x2)", rc_matrix_determinant(c) != -1.0); //assert that rc_matrix_t is sucessful when provided with valid input
                test_determinant("(2x2)", rc_matrix_determinant(c), 9);
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 3, 3);
                c.d[0][0] = 3;
                c.d[0][1] = 1;
                c.d[0][2] = 4;
                c.d[1][0] = 6;
                c.d[1][1] = 5;
                c.d[1][2] = 1;
                c.d[2][0] = 2;
                c.d[2][1] = 3;
                c.d[2][2] = 2;
                test("general error test (3x3)", rc_matrix_determinant(c) != -1.0); //assert that rc_matrix_t is sucessful when provided with valid input
                test_determinant("(3x3)", rc_matrix_determinant(c), 43);
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 4, 4);
                c.d[0][0] = 3;
                c.d[0][1] = 1;
                c.d[0][2] = 4;
                c.d[0][3] = 1;
                c.d[1][0] = 6;
                c.d[1][1] = 5;
                c.d[1][2] = 1;
                c.d[1][3] = 2;
                c.d[2][0] = 2;
                c.d[2][1] = 3;
                c.d[2][2] = 2;
                c.d[2][3] = 8;
                c.d[3][0] = 4;
                c.d[3][1] = 2;
                c.d[3][2] = 0.5;
                c.d[3][3] = 4.7;
                test("general error test (4x4)", rc_matrix_determinant(c) != -1.0); //assert that rc_matrix_t is sucessful when provided with two valid input
                test_determinant("(4x4)", rc_matrix_determinant(c), 361.1);
                rc_matrix_free(&c); //free memory
                sleep_ms(200);


                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
        }

        

        rc_matrix_symmetrize:hide_unused_error
        { //test the rc_matrix_t matrix symmetrise method [ rc_matrix_symmetrize ]
            pause();
            
            printf(CYN "\nTest the rc_matrix_t matrix symmetrise method [ rc_matrix_symmetrize ]" RST);
            
            { //subtest 1: invalid input
                printf(CYN "\n\tFirst subtest, invalid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);
                
                c.rows=1;c.cols=1;c.initialized=1;c.d=NULL; //set the values of the matrix such that the dimensions and initialized are valid but the pointer is null
                test("null pointer test", rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with a null pointer vector
                rc_matrix_free(&c); //free memory
                
                rc_matrix_zeros(&c, 1, 1);
                c.rows=1;c.cols=1;c.initialized=0; //set the values of the matrix such that the initialized is 0
                test("uninitialized test", rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with an uninitialized matrix
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=-1;c.cols=2; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (-1x2)", rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with invalid dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=0;c.cols=1; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test 0x1)", rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with invalid dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=6;c.cols=-1; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (6x-1)",  rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with  invalid, non square dims
                rc_matrix_free(&c); //free memory

                rc_matrix_zeros(&c, 1, 1);
                c.rows=7;c.cols=3; //set the values of the matrtix such that the dimensions are invalid
                test("invalid dims test (7x3)",  rc_matrix_symmetrize(&c) == -1); //assert that rc_matrix_t errors when provided with non square dims
                rc_matrix_free(&c); //free memory
            }


            { //subtest 2: valid input
                pause();

                printf(CYN "\n\tSecond subtest, valid input:" RST);
		        rc_matrix_t c;
                c = rc_matrix_empty();
                sleep_ms(200);

                rc_matrix_random(&c, 1, 1);
                test("general error test (1x1)",  rc_matrix_symmetrize(&c) != -1.0); //assert that rc_matrix_t is sucessful when provided with a valid vector
                test_symmetric("(1x1)", c);
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 2, 2);
                test("general error test (2x2)",  rc_matrix_symmetrize(&c) != -1.0); //assert that rc_matrix_t is sucessful when provided with a valid vector
                test_symmetric("(2x2)", c);
                rc_matrix_free(&c); //free memory

                rc_matrix_random(&c, 5, 5);
                test("general error test (5x5)",  rc_matrix_symmetrize(&c) != -1.0); //assert that rc_matrix_t is sucessful when provided with a valid vector
                test_symmetric("(5x5)", c);
                rc_matrix_free(&c); //free memory
                sleep_ms(200);

                printf(YLW "\n\tIf there were no test fails, this subtest has passed sucessfully!" RST);
            }

            sleep_ms(200);
            printf(YLW "\nIf there were no test fails, this test has passed sucessfully!" RST);
            
        }



        pause();
    }

    sleep_ms(250);
    printf(YLW "\n\nThere are no more tests!" RST);

return 0;
}
