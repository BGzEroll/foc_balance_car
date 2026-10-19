// Isolated diagnostic, included only by v24. Never prints while recording.
namespace sensor_record {
constexpr unsigned CAP=600;
struct Record {unsigned captured,imu,left,right;uint16_t left_raw,right_raw;int16_t acc[3],gyro[3],temp,pad;float angle[3],left_angle,right_angle,left_speed,right_speed;};
Record* buffer=nullptr;unsigned enabled=0,count=0,start_us=0,done_us=0,in_use=0;float gyro_bias[3]={};
bool owns_buffer(){return __atomic_load_n(&in_use,__ATOMIC_ACQUIRE)!=0;}
void task(void*){
 uint64_t last=0;
 while(true){
  if(__atomic_load_n(&enabled,__ATOMIC_ACQUIRE)){
   sensor::package p{};sensor::get_package(p);unsigned now=unsigned(esp_timer_get_time());
   if(now-start_us>6000000){done_us=now;__atomic_store_n(&enabled,0,__ATOMIC_RELEASE);}
   else if(p.imu.timestamp_us&&p.imu.timestamp_us!=last&&unsigned(p.imu.timestamp_us)-start_us<6000000){
    last=p.imu.timestamp_us;unsigned n=__atomic_load_n(&count,__ATOMIC_RELAXED);Record q{};
    q.captured=now;q.imu=unsigned(p.imu.timestamp_us);q.left=unsigned(p.left_encoder.timestamp_us);q.right=unsigned(p.right_encoder.timestamp_us);
    q.left_raw=p.left_encoder.raw_counts;q.right_raw=p.right_encoder.raw_counts;
    for(unsigned k=0;k<3;++k){q.acc[k]=p.imu.raw_acc[k];q.gyro[k]=p.imu.raw_gyro[k];q.angle[k]=p.imu.angle[k];if(!n)gyro_bias[k]=p.imu.gyro_bias[k];}
    q.temp=p.imu.raw_temperature;q.left_angle=p.left_encoder.angle_rad;q.right_angle=p.right_encoder.angle_rad;q.left_speed=p.left_encoder.speed_rad_s;q.right_speed=p.right_encoder.speed_rad_s;
    buffer[n]=q;__atomic_store_n(&count,n+1,__ATOMIC_RELEASE);
    if(n+1>=CAP){done_us=now;__atomic_store_n(&enabled,0,__ATOMIC_RELEASE);}
   }
  }
  vTaskDelay(1);
 }
}
void init(void* storage,size_t bytes){if(bytes>=sizeof(Record)*CAP)buffer=static_cast<Record*>(storage);if(buffer)ESP_ERROR_CHECK(xTaskCreatePinnedToCore(task,"sensor_record",4096,nullptr,2,nullptr,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM);}
bool command(const char* line,bool stopped){
 if(strncmp(line,"SREC",4))return false;
 if(!buffer){puts("SREC_ERROR allocation");return true;}
 if(!strcmp(line,"SRECSTART")){
  if(!stopped||enabled){puts("SREC_ERROR busy");return true;}
  count=0;done_us=0;start_us=unsigned(esp_timer_get_time());__atomic_store_n(&in_use,1,__ATOMIC_RELEASE);__atomic_store_n(&enabled,1,__ATOMIC_RELEASE);puts("SREC_START");return true;
 }
 if(!strcmp(line,"SRECDUMP")){
  if(!stopped||enabled){puts("SREC_ERROR busy");return true;}
  unsigned n=__atomic_load_n(&count,__ATOMIC_ACQUIRE);
  printf("SREC_META {\"version\":24,\"count\":%u,\"capacity\":%u,\"start_us\":%u,\"done_us\":%u,\"row_bytes\":%u,\"gyro_bias_rad_s\":[%.9f,%.9f,%.9f],\"acc_counts_per_g\":16384,\"gyro_counts_per_deg_s\":65.5,\"imu_rate_hz\":200,\"dlpf_cfg\":3,\"acc_complementary_weight\":0.02,\"encoder_speed_tf_s\":0.003}\n",n,CAP,start_us,done_us,unsigned(sizeof(Record)),gyro_bias[0],gyro_bias[1],gyro_bias[2]);
  puts("SREC_CSV captured_us,imu_us,left_us,right_us,left_raw,right_raw,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,temp_raw,pitch_rad,roll_rad,yaw_rad,left_angle_rad,right_angle_rad,left_speed_rad_s,right_speed_rad_s");
  for(unsigned i=0;i<n;++i){auto&q=buffer[i];printf("SREC_R %u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",q.captured,q.imu,q.left,q.right,q.left_raw,q.right_raw,q.acc[0],q.acc[1],q.acc[2],q.gyro[0],q.gyro[1],q.gyro[2],q.temp,q.angle[0],q.angle[1],q.angle[2],q.left_angle,q.right_angle,q.left_speed,q.right_speed);if(i%10==0)vTaskDelay(1);}
  puts("SREC_END");fflush(stdout);return true;
 }
 puts("SREC_ERROR command");return true;
}
}
