//
// Created by jwscoggins on 9/19/21.
//

#ifndef SXCHIPSET_TAGGEDCACHEADDRESS_H
#define SXCHIPSET_TAGGEDCACHEADDRESS_H
#include "MCUPlatform.h"
template<byte tagBits, byte totalBits = 32, byte offsetBits = 4, bool useSpecificTypeSizes = true>
union TaggedAddress {
    static constexpr auto NumLowestBits = offsetBits;
    static constexpr auto NumTagBits = tagBits;
    static constexpr auto NumRestBits = totalBits - (NumTagBits + NumLowestBits);
    static constexpr auto MaximumAddressSize = totalBits;
    static_assert((NumLowestBits + NumTagBits + NumRestBits) == MaximumAddressSize, "Too many or too few bits for this given tagged address!");
    static_assert((MaximumAddressSize >= 26) && (MaximumAddressSize <= 32), "Addresses cannot be smaller than 26 bits!");
    using TagType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumTagBits>, Address>;
    using RestType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumRestBits>, Address>;
    using LowerType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<NumLowestBits>, Address>;
    using AddressType = conditional_t<useSpecificTypeSizes, ClosestBitValue_t<MaximumAddressSize>, Address>;
    constexpr explicit TaggedAddress(Address value = 0) noexcept : base(value) { }
    constexpr explicit TaggedAddress(RestType key, TagType tag, LowerType offset = 0) noexcept : lowest(offset), tagIndex(tag), rest(key) { }
    void clear() noexcept { base = 0; }
    [[nodiscard]] constexpr auto getTagIndex() const noexcept { return tagIndex; }
    [[nodiscard]] constexpr auto getAddress() const noexcept { return base; }
    [[nodiscard]] constexpr auto getLowest() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getOffset() const noexcept { return lowest; }
    [[nodiscard]] constexpr auto getRest() const noexcept { return rest; }
    [[nodiscard]] constexpr auto getPSRAMChipId() const noexcept { return offset; }
    [[nodiscard]] constexpr auto getPSRAMIndex() const noexcept { return psramIndex; }
    [[nodiscard]] constexpr auto getPSRAMAddress() const noexcept { return psramIndex; }
    [[nodiscard]] constexpr auto getPSRAMAddress_High() const noexcept { return bytes_[2]; }
    [[nodiscard]] constexpr auto getPSRAMAddress_Middle() const noexcept { return bytes_[1]; }
    [[nodiscard]] constexpr auto getPSRAMAddress_Low() const noexcept { return bytes_[0]; }
    [[nodiscard]] TaggedAddress aligned() const noexcept {
        TaggedAddress result(base);
        result.lowest = 0;
        return result;
    }
    [[nodiscard]] bool restEqual(TaggedAddress other) const noexcept { return getRest() == other.getRest(); }
private:
    AddressType base;
    struct {
        LowerType lowest : NumLowestBits;
        TagType tagIndex : NumTagBits;
        RestType rest : NumRestBits;
    };
    struct {
        uint24_t psramIndex : 23;
        byte offset : 3;
    };
    byte bytes_[4];
};
#endif //SXCHIPSET_TAGGEDCACHEADDRESS_H
