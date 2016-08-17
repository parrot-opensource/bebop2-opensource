typedef struct
{
  unsigned int bus_address;
  unsigned int used;
  unsigned int size;
  void *ba;
  struct list_head list;
} hlinc;

