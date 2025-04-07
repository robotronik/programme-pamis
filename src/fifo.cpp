#include "fifo.h"
#include <stdlib.h> // Pour malloc et free

// Structure pour représenter chaque élément de la file FIFO


// Initialise une nouvelle file
Fifo *fifo_init(void)
{
    Fifo *fifo = (Fifo *)malloc(sizeof(Fifo));
    if (fifo == NULL)
    {
        return NULL; // Erreur d'allocation mémoire
    }
    fifo->tail = NULL;
    return fifo;
}

// Vérifie si la file est vide
bool fifo_isEmpty(const Fifo *fifo)
{
    return fifo->tail == NULL;
}

// Ajoute un mouvement à la fin de la file
void fifo_enqueue(Fifo *fifo, mvt_t deplacement)
{
    Node *newNode = (Node *)malloc(sizeof(Node));
    if (newNode == NULL)
    {
        // Gestion des erreurs mémoire si l'allocation échoue
        return;
    }

    newNode->deplacement = deplacement;

    if (fifo->tail == NULL)
    {
        newNode->next = newNode; // La queue pointe vers elle-même si elle est vide
        fifo->tail = newNode;
    }
    else
    {
        newNode->next = fifo->tail->next;
        fifo->tail->next = newNode;
        fifo->tail = newNode; // Mettre à jour le tail
    }
}

// Retire et retourne le premier mouvement de la file
mvt_t fifo_dequeue(Fifo *fifo)
{
    if (fifo_isEmpty(fifo))
    {
        // Retourner un mouvement nul si la file est vide
        return (mvt_t){DEPLACEMENT_NULL, NULL};
    }

    Node *head = fifo->tail->next;
    mvt_t result = head->deplacement;

    if (head == fifo->tail)
    {
        free(head); // Si un seul élément, vider la queue
        fifo->tail = NULL;
    }
    else
    {
        fifo->tail->next = head->next;
        free(head);
    }

    return result;
}

// Ajoute un mouvement en tête de la file
void fifo_addFirst(Fifo *fifo, mvt_t deplacement)
{
    Node *newNode = (Node *)malloc(sizeof(Node));
    if (newNode == NULL)
    {
        // Gestion des erreurs mémoire
        return;
    }

    newNode->deplacement = deplacement;

    if (fifo->tail == NULL)
    {
        newNode->next = newNode;
        fifo->tail = newNode;
    }
    else
    {
        newNode->next = fifo->tail->next;
        fifo->tail->next = newNode;
    }
}

// Retourne la longueur de la file
size_t fifo_length(const Fifo *fifo)
{
    if (fifo_isEmpty(fifo))
    {
        return 0;
    }

    size_t count = 0;
    Node *current = fifo->tail->next;

    do
    {
        count++;
        current = current->next;
    } while (current != fifo->tail->next);

    return count;
}
