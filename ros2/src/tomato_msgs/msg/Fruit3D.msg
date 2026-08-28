# 열매 하나 — **미터, header.frame_id 좌표계 기준**.
#
# 기존 스택의 vision.Fruit(픽셀 x,y + 면적)를 대체한다. 픽셀도 같이 싣는 이유는
# 화면에 그려 사람이 확인하기 위해서지, 그걸로 계산하라는 뜻이 아니다.

geometry_msgs/Point position   # 열매 중심. 단위 m. 좌표계는 header.frame_id
float32 radius                 # 반지름 추정 m. 깊이가 유효한 화소들로 잰 값
bool ripe                      # 빨강 = 수확 대상
float32 confidence             # 0..1

# --- 진단용 (계산에 쓰지 말 것) ---
uint16 u                       # 픽셀 중심 x
uint16 v                       # 픽셀 중심 y
uint32 pixels                  # 마스크 화소 수
uint32 depth_pixels            # 그중 **깊이가 유효했던** 화소 수.
                               # pixels에 비해 너무 적으면 그 점은 못 믿는다
                               # (D405는 빛나는 표면·가장자리에서 구멍이 난다)
float32 depth_spread           # 유효 깊이의 표준편차 m. 크면 잎/배경이 섞인 것
