#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#pragma pack(1)
void generateTableInt(char* buf, int rows, int cols, const char* title, const char* headings[], int data[rows][cols]) {
    char line[256] = {0};
    int title_len = strlen(title);
    int line_len = cols * 12;  // Each column is 12 chars wide

    // Top line
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);

    // Title (yellow color)
    sprintf(buf + strlen(buf), "|\033[33m %s%*s\033[0m|\n", title, line_len - title_len - 2, "");

    // Headings (blue color)
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|\033[34m  %s%*s\033[0m", headings[i], 9 - (int)strlen(headings[i]), "");
    }
    sprintf(buf + strlen(buf), "|\n");

    // Divider
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|%s", "-----------");
    }
    sprintf(buf + strlen(buf), "|\n");

    // Data
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sprintf(buf + strlen(buf), "| %9d ", data[i][j]);
        }
        sprintf(buf + strlen(buf), "|\n");
    }

    // Bottom line
    //sprintf(buf + strlen(buf), "|%s|\n", line);
}

void generateTableFloat(char* buf, int rows, int cols, const char* title, const char* headings[], float data[rows][cols]) {
    char line[256] = {0};
    int title_len = strlen(title);
    int line_len = cols * 12;  // Each column is 12 chars wide

    // Top line
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);

    // Title (yellow color)
    sprintf(buf + strlen(buf), "|\033[33m %s%*s\033[0m|\n", title, line_len - title_len - 2, "");

    // Headings (blue color)
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|\033[34m  %s%*s\033[0m", headings[i], 9 - (int)strlen(headings[i]), "");
    }
    sprintf(buf + strlen(buf), "|\n");

    // Divider
    for (int i = 0; i < cols; i++) {
        sprintf(buf + strlen(buf), "|%s", "-----------");
    }
    sprintf(buf + strlen(buf), "|\n");

    // Data
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sprintf(buf + strlen(buf), "|  %8.4f ", data[i][j]);
        }
        sprintf(buf + strlen(buf), "|\n");
    }

    // Bottom line
    //sprintf(buf + strlen(buf), "|%s|\n", line);
}

void generateBottomLine(char* buf, int cols) {
    char line[256] = {0};
    int line_len = cols * 12;  // Each column is 12 chars wide
    memset(line, '-', line_len-1);
    line[line_len] = '\0';
    sprintf(buf, "|%s|\n", line);
}