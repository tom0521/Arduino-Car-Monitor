#ifndef PROCESS_H
#define PROCESS_H

#include "list.h"

typedef void (*operation)();

/**
 * Structure for jobs to run
 * 
 * These jobs will be queued up as
 * needed. The func pointer is the
 * job that needs to be executed.
 */
struct process {
    operation func;    // Function to execute

    struct list_elem elem; // Needed to add to lists
};

#endif // PROCESS_H