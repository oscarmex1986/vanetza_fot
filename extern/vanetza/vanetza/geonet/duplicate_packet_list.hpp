#ifndef DUPLICATE_PACKET_LIST_HPP_T8JWKCKG
#define DUPLICATE_PACKET_LIST_HPP_T8JWKCKG

#include <vanetza/geonet/sequence_number.hpp>
#include <boost/circular_buffer.hpp>

namespace vanetza
{
namespace geonet
{

/**
 * Duplicate Packet List for a single source SO.
 * Those objects area meant as extension to LocationTableEntry.
 *
 * \see EN 302 636-4-1 v1.3.1 Annex A.2
 */
class DuplicatePacketList
{
public:
    DuplicatePacketList(unsigned elements);

    /**
     * Duplicate packet detection based on sequence number.
     *
     * Sequence number will be included in list afterwards.
     * \param sn sequence number
     * \return true if its a duplicate
     */
    bool check(SequenceNumber); // OAM Original check(SequenceNumber)

    /**
     * Retrieve duplicate packet counter
     * \param sn sequence number
     * \return number of duplicates seen for given sequence number
     */
    unsigned counter(SequenceNumber) const;

    bool checkReceived(SequenceNumber);

private:
    struct ListElement
    {
        ListElement(SequenceNumber);

        SequenceNumber sequence_number;
        unsigned counter;
        bool pendingRx;// OAM Flag que controla el pass-up a Facilities
        uint8_t hopCount; // OAM Guarda el valor de max hopcount 
    };

    ListElement* find(SequenceNumber);
    void flagReceived(SequenceNumber);


    boost::circular_buffer<ListElement> m_elements;
};

} // namespace geonet
} // namespace vanetza

#endif /* DUPLICATE_PACKET_LIST_HPP_T8JWKCKG */

