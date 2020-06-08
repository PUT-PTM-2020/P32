#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>   
#include<sys/ioctl.h>
#include<unistd.h>  
#include<errno.h>
#include <stdlib.h>

int socket_desc;
struct sockaddr_in server;
char *parray;
char nazw[3];

void odbior(){
    char buffer[80];
    bzero(buffer, 80);
    printf("\nCzekam na wiadomosc...\n");
    read(socket_desc, buffer, sizeof(buffer));
    printf("\nOdebrano: %s\n", buffer);
    bzero(buffer,80);
}

void odbid(){
    char buffer[3];
    bzero(buffer, 3);
    printf("\nCzekam na id + predkosc\n");
    read(socket_desc, buffer,sizeof(buffer));
    printf("\nOdebrano: %s\n", buffer);
    strncpy(nazw, buffer, 3);
}

void wysyl(char napis[10]){
    printf("\nWysylam napis...\n");
    send(socket_desc, napis, strlen(napis),0);
}

int receive_image(int socket)
{ // Start function 

int buffersize = 0, recv_size = 0,size = 0, read_size, write_size, packet_index =1,stat;
char tmpsize[5];

char imagearray[640],verify = '1';
FILE *image;

//Find the size of the image
do{
stat = read(socket, &tmpsize, sizeof(tmpsize));
}while(stat<0);

printf("\nOtrzymany img size: %s\n", tmpsize);

sscanf(tmpsize, "%d", &size);

printf("Packet received.\n");
printf("Packet size: %i\n",stat);
printf("Image size: %i\n",size);
printf(" \n");

char buffer[] = "Got it";

//Send our verification signal
/*do{
stat = write(socket, &buffer, sizeof(int));
}while(stat<0);*/

//printf("Reply sent\n");
printf(" \n");

char ext[] = ".raw";

char* name_with_extension;
name_with_extension = malloc(strlen(nazw)+1+4); /* make space for the new string (should check the return value ...) */
strcpy(name_with_extension, nazw); /* copy name into the new var */
strcat(name_with_extension, ext);

printf("\nZapisuje: %s\n", name_with_extension);

image = fopen(name_with_extension, "w");

if( image == NULL) {
printf("Error has occurred. Image file could not be opened\n");
return -1; }

//Loop while we have not received the entire file yet


int need_exit = 0;
struct timeval timeout = {20,0};

fd_set fds;
int buffer_fd, buffer_out;

while(recv_size < size) {
//while(packet_index < 2){

    FD_ZERO(&fds);
    FD_SET(socket,&fds);

    buffer_fd = select(FD_SETSIZE,&fds,NULL,NULL,&timeout);

    if (buffer_fd < 0)
       printf("error: bad file descriptor set.\n");

    if (buffer_fd == 0)
       printf("error: buffer read timeout expired.\n");

    if (buffer_fd > 0)
    {
        do{
               read_size = read(socket,imagearray, 640);
            }while(read_size <0);

            printf("Packet number received: %i\n",packet_index);
        printf("Packet size: %i\n",read_size);


        //Write the currently read data into our image file
         write_size = fwrite(imagearray,1,read_size, image);
         printf("Written image size: %i\n",write_size); 

             if(read_size !=write_size) {
                 printf("error in read write\n");    }


             //Increment the total number of bytes read
             recv_size += read_size;
             packet_index++;
             printf("Total received image size: %i\n",recv_size);
             printf(" \n");
             printf(" \n");
    }

}


  fclose(image);
  printf("Image successfully Received!\n");
  return 1;
  }


int main(int argc , char *argv[])
    {
    
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);

    if (socket_desc == -1) {
    printf("Could not create socket");
    }

    memset(&server,0,sizeof(server));
    server.sin_addr.s_addr = inet_addr("192.168.4.1");
    server.sin_family = AF_INET;
    server.sin_port = htons( 5200 );

    printf("\nLaczenie z fotoradarem...\n");
    //Connect to remote server
    if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0) {
    printf("%s", strerror(errno));
    close(socket_desc);
    puts("\nConnect Error\n");
    return 1;
    }
    printf("\nPolaczono...\n");
    char napis[10] = "cfg";
    char pobierz[10] = "dta";
    char predkosc[10];
    char iksdeki[10] = "xd";
    while(1){

        /*for (int i = 0; i < 3; i++){
            wysyl(napis);
            sleep(1);
        }*/
        //wysyl(napis);
        //sleep(1);
        //wysyl(predkosc);
        //break;
        //wysyl(iksdeki);
        //sleep(1);

        //odbior();

        //receive_image(socket_desc);

        printf("\nNapisz 1 aby podac predkosc, 2 aby pobrac zdjecia, lub 3 aby zamknac.\n");
        int decyzja;
        scanf("%d", &decyzja);

        if(decyzja == 1){
            printf("\nPodaj predkosc dwucyfrowa w km/h (0 na poczatku dla jednocyfrowych)\n");
            scanf("%s", predkosc);
            printf("\nPrzesylam podana predkosc: %s",predkosc);
            wysyl(napis);
            wysyl(predkosc);
        } else if(decyzja == 2){
            wysyl(pobierz);
            odbid();
            receive_image(socket_desc);
        } else if(decyzja == 3){
            printf("\nZamykam\n");
            break;
        } else {
            printf("\nNie ma takiej opcji\n");
        }
    }
    close(socket_desc);
    return 0;
}
