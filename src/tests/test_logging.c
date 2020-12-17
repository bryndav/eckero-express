#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

typedef struct data
{
    float heading;
    float pitch;
    int depth;
}DataSet;

int checkFile(const char*);
int writeCSVHeader(const char*);
int writeLogEntrie(const char*, DataSet);
int genRandomInt(int, int);
float genRandomFloat(float, float);
void updateDataValues (DataSet*);

int main()
{
    int result, log_val_i;
    int log_count = 100, count = 0;;
    float log_val_f;
    const char *filename = "data.log";
    DataSet some_set;

    srand(time(NULL));
    result = checkFile(filename);

    if (result != -1){
        printf("File exists, return code: %d\n", result);
    } else {
        printf("Could not find file, return code: %d\n", result);
        printf("Creating file...\n");
        result = writeCSVHeader(filename);
    }

    while(count < log_count){
        updateDataValues(&some_set);
        writeLogEntrie(filename, some_set);
        count++;
    }

    return 0;
}

int
checkFile(const char* path)
{
    int result;

    result = access(path, F_OK);

    return result;
}

int
writeCSVHeader(const char* path)
{
    const char *heading = "heading";
    const char *pitch = "pitch";
    const char *depth = "depth";

    FILE *f = fopen (path, "w");

    if (f == NULL)
        return -1;

    fprintf(f, "%s, %s, %s\n", heading, pitch, depth);
    fclose(f);

    return 0;
}

int
writeLogEntrie(const char *path, DataSet data)
{
    FILE *f = fopen(path, "a");
    fprintf(f, "%f, %f, %d\n", data.heading, data.pitch, data.depth);
    fclose(f);

    return 0;
}

int
genRandomInt (int upper,
              int lower)
{
    int return_val;

    return_val = (rand() % (upper - lower + 1)) + lower;

    return return_val;
}

float
genRandomFloat (float upper,
                float lower)
{
    float return_val;

    return_val = (((float)rand()/(float)(RAND_MAX)) * upper) + lower;

    return return_val;
}

void
updateDataValues (DataSet* data)
{
    data->heading = genRandomFloat(10.5, 3.5);
    data->depth = genRandomInt(640, 320);
    data->pitch = genRandomFloat(5.5, 1.0);
}
