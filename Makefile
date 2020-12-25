#SOURCE = $ 
TARGET = modem4g 
OBJECTS =  modem.o atchannel.o at_tok.o 
 
 
INCLUDES = -I ./h 
CFLAGS = -lpthread -Wall -c  
 

 
$(TARGET): $(OBJECTS)  
	$(CC) -lpthread $(OBJECTS) -o $(TARGET) 
#modem_up_1.o :  
#atchannel.o :atchannel.h at_tok.h 
#at_tok.o : at_tok.h log.h 
 
%.o: %.c 
	$(CC) $(INCLUDES) $(CFLAGS) $< -o $@
 
.PHONY: clean　　 
clean :
	rm  $(OBJECTS) $(TARGET) 
