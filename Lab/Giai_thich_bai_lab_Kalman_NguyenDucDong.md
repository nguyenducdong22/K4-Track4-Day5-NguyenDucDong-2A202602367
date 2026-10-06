# Giải thích bài Lab: Kalman Filter & Hợp nhất xác suất

**Sinh viên:** Nguyễn Đức Đông · **MSV:** 2A202602367
**Notebook:** K4-Track4-Day5-NguyenDucDong-2A202602367 (Google Colab)

---

## 1. Bài lab này nói về cái gì?

Bài toán chung: một chiếc xe/robot có **cảm biến nhiễu**. Ta muốn biết nó *thực sự* đang ở đâu và đi nhanh bao nhiêu. Bộ lọc Kalman giải quyết bằng cách lặp hai bước:

| Bước | Ý nghĩa | Công thức (dạng ma trận) |
|---|---|---|
| **Predict** (dự đoán) | Đẩy niềm tin về trạng thái tới thời điểm tiếp theo theo mô hình chuyển động; độ bất định tăng | `x⁻ = F x`, `P⁻ = F P Fᵀ + Q` |
| **Update** (cập nhật) | Trộn dự đoán với phép đo mới, mỗi bên được trọng số theo độ chính xác; độ bất định giảm | `ν = z − Hx⁻`, `S = HP⁻Hᵀ + R`, `K = P⁻HᵀS⁻¹`, `x = x⁻ + Kν`, `P = (I − KH)P⁻` |

Ý tưởng cốt lõi (Phần 2): **độ chính xác (1/σ²) cộng dồn**. Nguồn nào chính xác hơn thì có trọng số lớn hơn, và kết quả hợp nhất luôn chắc chắn hơn từng nguồn riêng lẻ.

## 2. Cấu trúc bài và điểm

| Phần | Nội dung | Chấm |
|---|---|---|
| 0–4 | Trung bình trượt, Gaussian, Kalman 1D, tinh chỉnh Q/R bằng NIS | Code có sẵn, chỉ chạy |
| 5.1, 5.2 | Ma trận F, H và lớp `KalmanFilter` | 25 đ |
| 6.1 | Vòng lặp hợp nhất LiDAR + radar + camera | 15 đ |
| 7.1 | Cập nhật có cổng χ² (loại outlier) | 10 đ |
| 9 | Nhiệm vụ Lynx-07: chẩn đoán cảm biến lỗi + báo cáo | 30 đ tự động + 20 đ báo cáo |
| 8 + sau giờ | EKF (bonus), ghi chú phụ | tối đa +5 |

## 3. Code từng bài và lý do

### Bài 5.1: `make_F`, `make_H`
- Trạng thái `x = [x, y, vx, vy]`. Mô hình vận tốc không đổi: `x ← x + vx·dt`, `y ← y + vy·dt`. Vì vậy `F = I` và thêm `F[0,2] = F[1,3] = dt`.
- Cảm biến chỉ đo vị trí nên `H` là ma trận 2×4 với `H[0,0] = H[1,1] = 1`.

### Bài 5.2: lớp `KalmanFilter`
- `predict`: `x = F@x`, `P = F@P@F.T + Q`.
- `update`: tính đổi mới `y`, `S`, độ lợi `K`, rồi cập nhật `x`, `P` và trả về `(y, S, K)`.
- Ô kiểm tra so sánh với bộ lọc 1 chiều của Phần 3: kết quả giống hệt, tức là bản ma trận chỉ là dạng tổng quát của bản vô hướng.
- Điểm hay nhất của phần này: **bộ lọc tự suy ra vận tốc** dù không cảm biến nào đo vận tốc, nhờ tương quan vị trí–vận tốc trong `P`.

### Bài 6.1: `run_fusion`
Với mỗi phép đo `(ts, name, z, H, R)` theo thứ tự thời gian:
1. `dt = ts − t_prev`; nếu `dt > 0` thì predict với `make_F(dt)` và `make_Q(dt, q)`.
2. Update với đúng `H` và `R` của cảm biến đó.
3. Lưu `(ts, x, P)` vào log.

Hợp nhất không cần toán mới: mỗi cảm biến chỉ cần có `H` và `R` riêng. Kết quả: track hợp nhất có RMSE **0.11 m**, tốt hơn LiDAR (0.25 m), radar (0.23 m) và camera (0.97 m). Khi LiDAR bị che, track hợp nhất vẫn giữ được sai số khoảng 0.13 m.

### Bài 7.1: `gated_update`
- Tính khoảng cách Mahalanobis `d² = νᵀS⁻¹ν` (dùng `np.linalg.solve`, không nghịch đảo tường minh).
- Nếu `d² > chi2.ppf(0.99, df=len(z))` (bằng 9.21 với 2 chiều) thì **loại phép đo và không đụng vào bộ lọc**; ngược lại gọi `kf.update`.
- Kết quả: bắt được 30/30 phép đo "ma", RMSE giảm từ 3.46 m xuống 0.28 m.

## 4. Phần 9: chẩn đoán Lynx-07 (dữ liệu riêng cho MSV 2A202602367)

### Số liệu đọc được
| Cảm biến | mean(NIS) | median(NIS) | residual trung bình |
|---|---|---|---|
| GPS | 3.12 | 1.99 | [−0.28, −0.41] m |
| UWB | 2.92 | 1.94 | [+0.53, +0.81] m |

### Lập luận loại lỗi
- **Không phải underrated noise:** loại lỗi này đẩy cả *median* NIS lên cao, nhưng ở đây median ≈ 2, đúng kỳ vọng của NIS 2 chiều.
- **Không phải outlier burst:** loại lỗi này làm mean NIS lên hàng chục, nhưng ở đây mean chỉ khoảng 3.
- **Là bias:** residual trung bình lệch rõ khỏi [0, 0], hai cảm biến lệch **ngược chiều nhau**, và |UWB| ≈ 2·|GPS|. Đây đúng là dấu hiệu khi bộ lọc bị kéo về giữa hai cảm biến theo tỉ lệ độ chính xác (GPS nặng gấp đôi UWB).

### Lập luận cảm biến nào bị lệch
Đây là điểm khó, và tôi muốn bạn hiểu rõ: **chỉ nhìn residual thì không phân biệt được GPS lệch hay UWB lệch**. Cả hai giả thuyết đều cho cùng tỉ lệ residual khoảng 1:2, vì bộ lọc chỉ "thấy" được *hiệu* bias giữa hai cảm biến. Tôi đã kiểm tra điều này bằng mô phỏng: tỉ lệ ≈ 1.95 cho cả hai trường hợp.

Vì vậy tôi dùng thêm một thông tin vật lý: **xe xuất phát tại gốc (0, 0)** (bộ lọc cũng khởi tạo ở đó). Giả thuyết đúng phải đưa điểm xuất phát (sau khi trừ bias) về gần gốc hơn. Tôi đã kiểm định phương pháp này trên 132 nhiệm vụ mô phỏng khác (không dùng log của bạn): nó đúng 127/132 lần (≈ 96%). Với log của bạn, phương pháp chỉ ra **UWB**.

Tôi **không** dùng cờ `_reveal_truth` trên dữ liệu của bạn. Kết luận dựa hoàn toàn trên số liệu đọc được.

**Kết luận:** `MY_DIAGNOSIS_SENSOR = "UWB"`, `MY_DIAGNOSIS_TYPE = "bias"`, `FIX_SENSOR = "UWB"`, `FIX_METHOD = "bias"`.

### Kết quả sau khi sửa
- Pooled mean NIS = **2.24** (median 1.42), đạt ngưỡng < 8.
- 1σ vị trí cuối ≈ **0.40 m**; bán kính 95% ≈ **0.69 m**.

⚠️ **Rủi ro cần biết:** phần *loại lỗi* (bias) tôi rất chắc. Phần *cảm biến nào* dựa trên giả định xe xuất phát tại gốc, nên có khoảng 4% khả năng sai. Nếu sai cảm biến, bạn vẫn được 8/15 điểm ở mục 9.1, và NIS vẫn đạt.

## 5. Phần bonus và sau giờ
- **Bài 8.1 (EKF):** `h_rb` trả về `[khoảng cách, góc]`; Jacobian `H_rb` có hàng 1 là `[dx/r, dy/r, 0, 0]` và hàng 2 là `[−dy/r², dx/r², 0, 0]`. Kết quả khớp sai phân hữu hạn; EKF giảm RMSE từ 1.11 m xuống 0.56 m.
- **Cửa sổ hiệu chỉnh 0–20 s:** đã bật `RUN_OPTIONAL_93 = True`. Cửa sổ đầu cho cùng dấu vân tay bias; NIS vận hành sau giây 20 là 2.24.
- Đã điền đủ các ghi chú sau giờ (cửa sổ hiệu chỉnh, suy luận khi GPS mất tín hiệu 15 s).

## 6. Bạn nên tự nắm được gì
1. Kalman = predict (bất định tăng) + update (bất định giảm); K là "núm xoay độ tin cậy".
2. Q và R là những phát biểu trung thực về mức độ nghi ngờ; NIS ≈ số chiều phép đo nghĩa là bộ lọc nhất quán.
3. Hợp nhất nhiều cảm biến = một bộ lọc + nhiều bộ `(z, H, R)`, xử lý theo thứ tự thời gian.
4. Dấu vân tay lỗi: bias → residual lệch; nhiễu bị đánh giá thấp → median NIS cao; outlier → mean cao nhưng median bình thường.
5. Bias tương đối giữa hai cảm biến **không quan sát được tuyệt đối** nếu không có mốc tham chiếu. Đây là một hạn chế quan trọng nên nhắc trong báo cáo.
