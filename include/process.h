#ifndef PROCESS_H
#define PROCESS_H

#include "list.h"

typedef int8_t (*operation)(uint8_t arg);

/**
 * Structure for jobs to run
 * 
 * These jobs will be queued up as
 * needed. The func pointer is the
 * job that needs to be executed.
 */
struct process {
    operation func;         // Function to execute
    uint8_t arg;            // Function argument
    struct list_elem elem;  // Needed to add to lists
};

#endif // PROCESS_H