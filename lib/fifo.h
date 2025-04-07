

#ifndef FIFO_H
#define FIFO_H

#include "motor.h"  // Pour inclure les définitions de mvt_t et autres structures
typedef struct Node
{
    mvt_t deplacement;
    struct Node *next;
} Node;


typedef struct Fifo {
    Node* tail;
} Fifo;


// Prototypes des fonctions de la FIFO
Fifo* fifo_init(void);
void fifo_enqueue(Fifo* fifo, mvt_t deplacement);
mvt_t fifo_dequeue(Fifo* fifo);
void fifo_addFirst(Fifo* fifo, mvt_t deplacement);
size_t fifo_length(const Fifo* fifo);
bool fifo_isEmpty(const Fifo* fifo);

#endif
