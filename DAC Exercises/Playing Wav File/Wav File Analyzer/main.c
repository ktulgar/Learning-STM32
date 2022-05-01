#include <stdio.h>

int main() {

    FILE *wavFile = fopen("E:\\Wav File Analyzer\\recording.wav","rb");
    FILE *dac = fopen("dac_8bit.txt","w");
    fseek(wavFile,20,SEEK_SET);
    int data;
    fread(&data,2,1,wavFile);
    printf("Audio Format: %d\n",data);
    data = 0;
    fread(&data,2,1,wavFile);
    printf("Number Of Channels: %d\n",data);
    data = 0;
    fread(&data,4,1,wavFile);
    printf("Sample Rate: %d\n",data);
    data = 0;
    fread(&data,4,1,wavFile);
    printf("Byte Rate: %d\n",data);
    data = 0;
    fread(&data,2,1,wavFile);
    printf("Block Align: %d\n",data);
    data = 0;
    fread(&data,2,1,wavFile);
    printf("Bits Per Sample: %d\n",data);
    fseek(wavFile,40,SEEK_SET);
    data = 0;
    fread(&data,4,1,wavFile);
    printf("Size: %d\n",data);

    // Rest code only works  for unsigned 8-bit mono channel wav files.

    int x = 0;
    for(int i = 0 ; i < data ; i++) {
        uint8_t value;
        fread(&value,1,1,wavFile);
        fprintf(dac,"%d,",value);
        x++;
        if(x == 20) {
            fprintf(dac,"\n");
            x=0;
        }
    }



    fclose(wavFile);
    fclose(dac);

}
