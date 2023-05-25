#include <OneButton.h>

int BT1 = 2;
int BT2 = 4;
int LED = 3;

OneButton Button_1(BT1, true);
OneButton Button_2(BT2, true);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(BT1, INPUT_PULLUP);
  pinMode(BT2, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  
  Button_1.attachClick(one_click_bt1);                    //nhấn 1 lần rồi nhả lệnh sẽ được kích hoạt
  Button_1.attachDoubleClick(double_click_bt1);           //khi nhấn liên tục 2 lần lệnh sẽ được kích hoạt
  Button_1.attachLongPressStart(long_press_bt1);          //kich hoạt lệnh khi nhấn giữ 1s
  // Button_1.attackDuringLongPress(during_long_press_bt1);  //khi thực hiện xong lệnh trong chế độ nhấn giữ nếu còn nhất nút thì mới thực hiện nhấn giữ lâu
  Button_1.attachLongPressStop(release_button_bt1);       //kích hoạt lệnh khi nút nhấn được thả ra, áp dụng cho trường hợp nhấn giữ
  Button_1.attachMultiClick(multi_click_bt1);             //thực hiện lệnh sau khi nhiều lần nhấn (nhấn > 2 lần)

  Button_2.attachClick(one_click_bt2);                    //nhấn 1 lần rồi nhả lệnh sẽ được kích hoạt
  Button_2.attachDoubleClick(double_click_bt2);           //khi nhấn liên tục 2 lần lệnh sẽ được kích hoạt
  Button_2.attachLongPressStart(long_press_bt2);          //kich hoạt lệnh khi nhấn giữ 1s
  // Button_2.attackDuringLongPress(during_long_press_bt2);  //khi thực hiện xong lệnh trong chế độ nhấn giữ nếu còn nhất nút thì mới thực hiện nhấn giữ lâu
  Button_2.attachLongPressStop(release_button_bt2);       //kích hoạt lệnh khi nút nhấn được thả ra, áp dụng cho trường hợp nhấn giữ
  Button_2.attachMultiClick(multi_click_bt2);             //thực hiện lệnh sau khi nhiều lần nhấn (nhấn > 2 lần)
}

void loop() {
  // put your main code here, to run repeatedly:
  Button_1.tick();      //kiểm tra trạng thái nút nhấn
  Button_2.tick();
  delay(50);
}

void one_click_bt1()
{
  Serial.println("one_click_bt1");
}
void double_click_bt1()
{
  Serial.println("double_click_bt1");
}
void long_press_bt1()
{
Serial.println("long_press_bt1");
}
// void during_long_press_bt1()
// {
//   Serial.println("during_long_press_bt1");
// }
void release_button_bt1()
{
  Serial.println("release_button_bt1");
}
void multi_click_bt1()
{
  Serial.println("multi_click_bt1");
}

/*>>>>>>>BUTTON 2<<<<<<<*/
void one_click_bt2()
{
  Serial.println("one_click_bt2");
}
void double_click_bt2()
{
  Serial.println("double_click_bt2");
}
void long_press_bt2()
{
Serial.println("long_press_bt2");
}
// void during_long_press_bt2()
// {
//   Serial.println("during_long_press_bt2");
// }
void release_button_bt2()
{
  Serial.println("release_button_bt2");
}
void multi_click_bt2()
{
  Serial.println("multi_click_bt2");
}