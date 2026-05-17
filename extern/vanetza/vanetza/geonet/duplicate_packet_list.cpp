#include <vanetza/geonet/duplicate_packet_list.hpp>
#include <cassert>


namespace vanetza
{
namespace geonet
{

DuplicatePacketList::DuplicatePacketList(unsigned elements) :
    m_elements(elements)
{
    assert(m_elements.size() == 0);
}

bool DuplicatePacketList::check(SequenceNumber sn)
{
    ListElement* element = find(sn);
    if (element) {
        ++element->counter;
        //if(element->pendingRx){
            return true;    
        //} else {
        //    flagReceived(sn);
        //    return true;
        //}
        
    } else {
        m_elements.push_back(ListElement { sn });
        return false;
    }
}

unsigned DuplicatePacketList::counter(SequenceNumber sn) const
{
    for (auto& element : m_elements) {
        if (element.sequence_number == sn) {
            return element.counter;
        }
    }
    return 0;
}

DuplicatePacketList::ListElement* DuplicatePacketList::find(SequenceNumber sn)
{
    for (auto& element : m_elements) {
        if (element.sequence_number == sn) {
            return &element;
        }
    }
    return nullptr;
}

//OAM Proceso que cambia el flag del elemento encontrado de false a true
void DuplicatePacketList::flagReceived(SequenceNumber sn)
{
    for (auto& element : m_elements) {
        if (element.sequence_number == sn) {
            element.pendingRx = true;
        }
    }
}

bool DuplicatePacketList::checkReceived(SequenceNumber sn)
{
    ListElement* element = find(sn);
    if (element) {
        if(element->pendingRx){
            return true;    
        } else {
            flagReceived(sn);
            return false;
        }
        
    } else {
        return false;
    }

}

DuplicatePacketList::ListElement::ListElement(SequenceNumber sn) :
    sequence_number(sn), counter(1), pendingRx(false)
{
}

} // namespace geonet
} // namespace vanetza
