#ifndef TABLE_GENERATOR_H
#define TABLE_GENERATOR_H

/**
 * @file print_tables.h
 * @brief Functions for generating formatted tables with integer or float data.
 */

/**
 * @brief Generates a formatted table with integer data.
 *
 * @param[out] buf The output buffer for the table string.
 * @param[in] rows The number of rows in the table.
 * @param[in] cols The number of columns in the table.
 * @param[in] title The title of the table.
 * @param[in] headings The headings for each column.
 * @param[in] data The integer data for the table.
 */
void generateTableInt(char* buf, int rows, int cols, const char* title, const char* headings[], int data[rows][cols]);

/**
 * @brief Generates a formatted table with floating-point data.
 *
 * @param[out] buf The output buffer for the table string.
 * @param[in] rows The number of rows in the table.
 * @param[in] cols The number of columns in the table.
 * @param[in] title The title of the table.
 * @param[in] headings The headings for each column.
 * @param[in] data The floating-point data for the table.
 */
void generateTableFloat(char* buf, int rows, int cols, const char* title, const char* headings[], float data[rows][cols]);

void generateBottomLine(char* buf, int cols);
#endif /* TABLE_GENERATOR_H */
