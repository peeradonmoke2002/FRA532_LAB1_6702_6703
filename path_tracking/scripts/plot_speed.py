#!/usr/bin/python3
import csv
import matplotlib.pyplot as plt

def main():
    csv_filename = "speed_data.csv"
    times = []
    speeds = []

    # อ่านข้อมูลจาก CSV
    try:
        with open(csv_filename, "r") as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader)  # ข้าม header
            for row in reader:
                times.append(float(row[0]))
                speeds.append(float(row[1]))
    except FileNotFoundError:
        print(f"❌ ไม่พบไฟล์ '{csv_filename}' เลย! รัน node บันทึกข้อมูลก่อนนะครับ")
        return

    # พล็อตกราฟเส้น (ไม่มีจุด)
    plt.figure(figsize=(10, 6))
    plt.plot(times, speeds, 'r-', label="Speed (m/s)")  # ใช้ '-' เพื่อให้เป็นเส้นเรียบ
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")
    plt.title("Speed vs Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # แสดงกราฟ
    plt.show()

if __name__ == "__main__":
    main()
