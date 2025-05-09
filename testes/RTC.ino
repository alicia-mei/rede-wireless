#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

struct tm data;//Cria a estrutura que contem as informacoes da data.

void setup()
{
  timeval tv;//Cria a estrutura temporaria para funcao abaixo.
  tv.tv_sec = 	1746791026;//Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
  settimeofday(&tv, NULL);//Configura o RTC para manter a data atribuida atualizada.

}

void loop(){
  time_t tt = time(NULL);//Obtem o tempo atual em segundos. Utilize isso sempre que precisar obter o tempo atual
  data = *gmtime(&tt);//Converte o tempo atual e atribui na estrutura
  
  char data_formatada[64];
  strftime(data_formatada, 64, "%d/%m/%Y %H:%M:%S", &data);//Cria uma String formatada da estrutura "data"


  printf("\nUnix Time: %d\n", int32_t(tt));//Mostra na Serial o Unix time
  printf("Data formatada: %s\n", data_formatada);//Mostra na Serial a data formatada
  delay(5000);
}
