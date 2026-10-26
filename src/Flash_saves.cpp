#include "Flash_saves.h"
#include "ch32v20x.h"
#include "ch32v20x_flash.h"
#include "CRC16.h"

#include <string.h>

namespace
{
constexpr uint32_t kPageAddr = 0x0800F000;
constexpr uint32_t kSlotSize = 32;
constexpr uint32_t kSlotCount = 128; // 128 * 32 = 4096
constexpr uint32_t kMagic = 0x424D4355; // "BMCU"
constexpr uint16_t kVersion = 2;

struct SettingsRecord
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint16_t seq;
    uint16_t crc;
    Settings cfg;
    uint8_t pad[kSlotSize - 12 - sizeof(Settings)];
};
static_assert(sizeof(SettingsRecord) == kSlotSize, "SettingsRecord layout changed");

Settings g_settings;
bool g_loaded = false;
bool g_dirty = false;
uint16_t g_seq = 0;
uint16_t g_last_slot = 0;

uint16_t settings_crc(const Settings &cfg)
{
    CRC16 crc;
    crc.reset(0x1021, 0x913D, 0, false, false);
    crc.restart();
    crc.add((const uint8_t *)&cfg, sizeof(cfg));
    return crc.calc();
}

bool slot_is_blank(uint32_t index)
{
    const volatile uint32_t *p = (const volatile uint32_t *)(kPageAddr + index * kSlotSize);
    for (int i = 0; i < (int)(kSlotSize / 4); i++)
    {
        if (p[i] != 0xFFFFFFFF)
            return false;
    }
    return true;
}

bool record_is_valid(const SettingsRecord &rec)
{
    return rec.magic == kMagic &&
           rec.version == kVersion &&
           rec.size == sizeof(Settings) &&
           rec.crc == settings_crc(rec.cfg);
}

bool flash_erase_page(uint32_t address)
{
    const uint32_t mstatus = __get_MSTATUS();
    bool ok = true;
    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);
    if (FLASH_ErasePage(address) != FLASH_COMPLETE)
        ok = false;
    FLASH_Lock();
    __set_MSTATUS(mstatus);
    return ok;
}

bool flash_program_verify(uint32_t address, const void *buf, uint32_t length)
{
    const uint16_t *src = (const uint16_t *)buf;
    const uint32_t mstatus = __get_MSTATUS();
    bool ok = true;

    __disable_irq();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_WRPRTERR);

    for (uint32_t i = 0; ok && i < length / 2; i++)
    {
        if (FLASH_ProgramHalfWord(address + i * 2, src[i]) != FLASH_COMPLETE)
            ok = false;
    }
    for (uint32_t i = 0; ok && i < length / 2; i++)
    {
        if (((const volatile uint16_t *)address)[i] != src[i])
            ok = false;
    }

    FLASH_Lock();
    __set_MSTATUS(mstatus);
    return ok;
}

bool write_slot(uint16_t slot, uint16_t seq, const Settings &cfg)
{
    if (slot >= kSlotCount)
        return false;

    SettingsRecord rec{};
    rec.magic = kMagic;
    rec.version = kVersion;
    rec.size = sizeof(Settings);
    rec.seq = seq;
    rec.crc = settings_crc(cfg);
    rec.cfg = cfg;

    return flash_program_verify(kPageAddr + slot * kSlotSize, &rec, sizeof(rec));
}
} // namespace

bool Settings_load()
{
    if (g_loaded)
        return true;
    g_loaded = true;

    SettingsRecord best;
    bool have_best = false;
    uint16_t best_seq = 0;
    uint16_t last_used = 0;
    bool any_used = false;

    for (uint16_t slot = 0; slot < kSlotCount; slot++)
    {
        if (!slot_is_blank(slot))
        {
            last_used = slot;
            any_used = true;
        }

        SettingsRecord rec;
        memcpy(&rec, (const void *)(kPageAddr + slot * kSlotSize), sizeof(rec));
        if (!record_is_valid(rec))
            continue;

        if (!have_best || (int16_t)(rec.seq - best_seq) > 0)
        {
            best = rec;
            best_seq = rec.seq;
            have_best = true;
        }
    }

    if (have_best)
    {
        g_settings = best.cfg;
        g_seq = best.seq;
        g_last_slot = any_used ? last_used : 0;
        g_dirty = false;
        return true;
    }

    // 不保留旧配置：整页擦除，写默认 Settings。
    g_settings = Settings{};
    g_seq = 1;
    g_last_slot = 0;
    g_dirty = false;

    if (!flash_erase_page(kPageAddr))
        return false;
    if (!write_slot(0, g_seq, g_settings))
        return false;
    return true;
}

Settings &Settings_get()
{
    return g_settings;
}

bool Settings_save()
{
    if (!g_loaded)
        Settings_load();

    uint16_t target = (uint16_t)(g_last_slot + 1);
    if (target >= kSlotCount)
    {
        if (!flash_erase_page(kPageAddr))
            return false;
        target = 0;
    }

    const uint16_t seq = (uint16_t)(g_seq + 1);
    if (!write_slot(target, seq, g_settings))
        return false;

    g_last_slot = target;
    g_seq = seq;
    g_dirty = false;
    return true;
}

void Settings_mark_dirty()
{
    g_dirty = true;
}

bool Settings_is_dirty()
{
    return g_dirty;
}
