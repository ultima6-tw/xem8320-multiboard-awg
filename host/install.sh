#!/usr/bin/env bash
set -euo pipefail

# install.sh -- XEM8320 + Si5332-6EX-EVB 一次性 Linux 機器設定
#
# 目的：讓這台 Linux 機器插上 Opal Kelly XEM8320 板子 / Si5332-6EX-EVB
# 之後就能直接跑 host/ 底下的 Python 腳本（不需要 sudo、不需要每次手動
# 加 udev 規則）。只需要在新機器上跑這支 script 一次，之後就是
# plug-and-play——跟 Opal Kelly 官方 FrontPanel SDK 本來就會做的事一樣，
# 只是這裡把 Si5332 EVB 那顆也一起包進來。
#
# 需要 sudo 密碼（複製 udev 規則 + reload），必須在有 TTY 的終端機
# 直接執行這支 script（不能透過非互動方式跑，sudo 會讀不到密碼）。
#
# 這支 script 支援兩種佈局，會自動判斷用哪一種找 FrontPanel SDK tgz：
#   1. 獨立打包版（build_linux_bundle.sh 產生的 .tar.gz 解壓後）：
#      跟自己同一層有 sdk/FrontPanel-*.tgz
#   2. repo 內開發版（在 Claude/Projects/FPGA/awg-test-step-16/host/ 直接跑）：
#      去 sibling 專案 Projects/FPGA/xem8320-awg/ 找同一份 tgz
# udev 規則兩種佈局都一樣放在自己同一層的 udev/ 底下。
#
# 用法：
#   ./install.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTPANEL_EXTRACT_DIR="$HOME/Vivado/FrontPanel-Ubuntu-6.0.0"
SI5332_RULE_SRC="$SCRIPT_DIR/udev/61-si5332-evb.rules"

if [ "$(uname)" != "Linux" ]; then
    echo "[X] 這支 script 只適用 Linux（udev 規則是 Linux 專屬機制）" >&2
    echo "    Windows 請參考 windows.md；macOS 通常不需要這一步。" >&2
    exit 1
fi

echo "=== 1/4: 解壓官方 FrontPanel Linux SDK（如果還沒解壓過）==="
FRONTPANEL_TGZ=""
for candidate in \
    "$SCRIPT_DIR"/sdk/FrontPanel-Ubuntu-*-x64-*.tgz \
    "$SCRIPT_DIR"/../../xem8320-awg/FrontPanel-Ubuntu-*-x64-*.tgz
do
    if [ -f "$candidate" ]; then
        FRONTPANEL_TGZ="$candidate"
        break
    fi
done
if [ -z "$FRONTPANEL_TGZ" ]; then
    echo "[X] 找不到 FrontPanel SDK tgz（找過 $SCRIPT_DIR/sdk/ 跟" >&2
    echo "    $SCRIPT_DIR/../../xem8320-awg/ 兩個位置）" >&2
    exit 1
fi
echo "  使用 SDK：$FRONTPANEL_TGZ"
if [ ! -d "$FRONTPANEL_EXTRACT_DIR" ]; then
    mkdir -p "$FRONTPANEL_EXTRACT_DIR"
    tar xzf "$FRONTPANEL_TGZ" -C "$FRONTPANEL_EXTRACT_DIR" --strip-components=1
    echo "  已解壓到 $FRONTPANEL_EXTRACT_DIR"
else
    echo "  已存在，跳過：$FRONTPANEL_EXTRACT_DIR"
fi

OK_RULE_SRC=$(find "$FRONTPANEL_EXTRACT_DIR" -name "60-opalkelly.rules" | head -1)
if [ -z "$OK_RULE_SRC" ]; then
    echo "[X] 找不到 60-opalkelly.rules，SDK 解壓可能有問題" >&2
    exit 1
fi

echo
echo "=== 2/4: 安裝 udev 規則（Opal Kelly + Si5332 EVB），需要 sudo 密碼 ==="
sudo cp "$OK_RULE_SRC" /etc/udev/rules.d/60-opalkelly.rules
sudo cp "$SI5332_RULE_SRC" /etc/udev/rules.d/61-si5332-evb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  udev 規則安裝完成：/etc/udev/rules.d/60-opalkelly.rules, 61-si5332-evb.rules"

echo
echo "=== 3/4: 安裝 Python 相依套件（ok / pyusb）==="
OK_WHEEL=$(find "$FRONTPANEL_EXTRACT_DIR" -name "ok-*.whl" | head -1)
if [ -z "$OK_WHEEL" ]; then
    echo "[X] 找不到 FrontPanel Python wheel" >&2
    exit 1
fi
pip3 install --user --upgrade "$OK_WHEEL"
pip3 install --user --upgrade pyusb

echo
echo "=== 4/4: 驗證 ==="
python3 -c "import ok; print('  ok module OK:', ok.__file__)"
python3 -c "import usb.core; print('  pyusb OK')"

echo
echo "[OK] 安裝完成。插上 XEM8320 板子或 Si5332-6EX-EVB 應該就能直接用，"
echo "     不需要 sudo。如果裝置本來就插著，建議重新拔插一次讓新的"
echo "     udev 規則生效。"
